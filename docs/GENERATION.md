# Procedural World Generation

This document states the problem of generating a coherent 3D scene (terrain,
roads, railways, water, bridges, tunnels, buildings) from 2D map data and an
elevation model, and specifies the design that solves it. It owns the
*vertical* model — heights, structures, the engineered ground, and the order in
which features are allowed to influence one another.

Companions: `docs/ROADS.md` owns the horizontal road surface (widths, junction
areas, markings) built on top of this; `docs/GROUND.md` details the ground
imprint and its meshes; `docs/VERIFICATION.md` owns the harness that measures
an emitted archive against §7.

Every claim in §7 is stated as a predicate with a check in §8. A design
statement with no check is a wish, not a specification.

---

## 1. What is being modeled

The scene is a **built landscape**: natural terrain reshaped by engineering,
with networks laid over it, structures where a network and the ground disagree,
and buildings founded on the result. Five observations drive everything else.

**1. The ground is the shared substrate.** Every feature is defined relative to
the ground: roads lie on it, buildings are founded on it, bridges stand above
it, tunnels pass under it. Scene coherence reduces largely to every generator
agreeing on *one* ground surface.

**2. The ground is partly man-made.** Embankments, cuttings, portals, retaining
walls, and building platforms all reshape the terrain. A generator that treats
the DEM as read-only cannot express them; the ground must be an *output* of
generation, not only an input.

**3. Each network holds an engineered geometry.** A motorway is built to ≤ 6 %,
a mainline railway to ≤ 3 %, a funicular to a single constant gradient, a
residential street to whatever hill it sits on. The alignment stays regular
even where the terrain under it is not.

**4. A structure exists exactly where a network's geometry and something else
disagree.** That "something else" is one of two things, and each calls for
different information:

- **Terrain.** A valley that dips below the alignment calls for a viaduct; a
  hill that rises above it calls for a tunnel. The profile against the terrain
  sets the structure's vertical position: the deck height is the grade line,
  and the portals sit where the alignment pierces the surface.
- **Another feature.** A road crosses another road, a railway, a canal, or a
  navigable river, often over flat ground. The terrain says nothing: clearance
  over the crossed feature sets the height, and the ground must be reshaped
  (approach embankments, retaining walls) to lift the road there. The same
  split holds underground: a mountain tunnel reconciles the alignment with the
  terrain; an urban underpass reconciles it with a crossing feature.

**5. The built world has a construction order, and it is recoverable from
feature class.** The river was there before anything. The railway was surveyed
to a standard and cut through what stood in the way. The street was laid on
what remained. The footpath was worn onto the finished surface. When two
features cannot both have the height they want, the junior one yields
*entirely* — it does not meet the senior one halfway. This ordering is the
organising principle of the whole model (§4).

Buildings only look simpler. A footprint on sloped ground has no single base
elevation: the structure must meet the ground on every side, with a foundation
reaching downhill or a platform cut into the slope. Roof forms must be
synthesized from sparse attributes. And at every zoom the buildings must sit on
the *rendered* terrain, which differs per LOD.

---

## 2. The data and what it does not say

### 2.1 Vector map data (Overture / OSM)

**Networks** are 2D centerlines with class attributes and linearly referenced
level annotations: fractional spans tagged with a small integer level (positive
= elevated, negative = below ground, 0 = grade). Six caveats:

- **Levels are ordinals, not heights.** `level = 1` means "above whatever is
  level 0 here"; `level = -5` does not mean 5 units deep. Crossing bridges at
  +1 and +2 encode an *ordering* that must be resolved into heights.
- **No elevations anywhere.** Not on nodes, not on structures.
- **Span boundaries are not registered to the terrain.** Where a tunnel
  annotation ends is where a mapper split it, not where the road emerges from
  the hillside. Annotation edges of adjacent structures routinely miss each
  other by metres, leaving phantom at-grade slivers.
- **A structure is not an entity.** One physical viaduct may be split across
  many segments, each with its own annotation; one segment may run at grade,
  climb a bridge, dive into a tunnel, and return to grade. That connectivity
  must come from the graph.
- **Crossings are implicit.** No link connects an overpass to the feature it
  crosses; the crossing must be found geometrically (2D intersection plus level
  order).
- **Dual carriageways are separate segments** that each claim to be a bridge,
  though the physical viaduct is one structure.

**Buildings** are 2D footprints (with interior rings) and sparse attributes:
height or floor count for some, roof shape for fewer, nothing for many. No
foundation or terrace geometry, no relation to the ground.

**Water** is polygons and centerlines with no surface elevations. A lake must
come out flat; a river must descend monotonically; neither is stated.

### 2.2 What the data *does* say: class is the authority signal

The class taxonomy is the one place the data records observation 5, and it is
richer than a single road-class enum can hold. Measured over a Swiss extract:

| Modality | Class | Segments |
|----------|-------|---------:|
| rail | `standard_gauge` | 29,997 |
| rail | `unknown` | 5,194 |
| rail | `narrow_gauge` | 4,509 |
| rail | `tram` | 1,712 |
| rail | `funicular` | 280 |
| rail | `light_rail`, `subway`, `monorail` | 180 |
| road | engineered (`motorway`…`tertiary`) | 224,996 |
| road | local (`residential`, `service`, `unclassified`, …) | 1,335,714 |
| road | draped (`track`, `footway`, `path`, `steps`, `cycleway`, …) | 1,380,422 |
| water | lines | 745 |

Two facts in that table shape §4. **Nearly half the road network (46.9 %) is
footpath-grade** — features that must never influence anything. And **rail is
not one thing**: a tram lies *on* a street while a funicular has its own
formation and a constant gradient, so a taxonomy keyed on modality alone gets
those two wrong in opposite directions.

The key for every prior and every stratum decision is therefore **(modality,
class)**, never a single flat enum.

### 2.3 The DEM

A ground-surface raster whose resolution ranges from 30 m (global) to 0.5 m
(national lidar):

- A DSM-flavored source may already *contain* the bridges, buildings, and
  embankments the generator is trying to synthesize; a DTM under a viaduct
  shows the ravine floor. Either way, **the DEM near a structure is the least
  trustworthy place to read a feature's own height**; at-grade stretches, not
  structure spans, must anchor any reconstructed profile.
- What the client renders is not the DEM but a decimated terrain mesh per zoom.
  "Sits on the ground" must hold against the *rendered* mesh at each zoom, or
  roads swim through hillsides and buildings hover.

### 2.4 What no data source provides

Pier positions, deck cross-sections, structure type, abutments, portal
architecture, retaining walls, foundation and terrace geometry, roof details,
actual built clearances. All of it is synthesized from priors (§9). The bar is
met by getting the *vertical relationships and contacts* right, not by
architectural detail.

---

## 3. Why this is hard

**D1. The vertical geometry is under-determined.** The data gives topology; the
render needs geometry. The missing information must be inferred from the
terrain plus engineering priors, and the inference is a constrained fit whose
constraints ignore feature boundaries: a deck height depends on anchors
hundreds of metres away and on the feature it crosses.

**D2. The model is global, the tiles are local.** A viaduct's grade line is a
function of kilometres of context; a tile is a small window. Any scheme that
lets a tile infer heights from its own window will fit a different answer than
its neighbour. This forces resolving the model *once, globally, before tiling*.

**D3. The ground is an output.** An overpass approach without an embankment
floats over flat grass; an underpass without a cut vanishes into the ground
plane; a building on a steep slope gapes downhill. The terrain meshes must
respond to the solved model.

**D4. Features interact, and the interaction is asymmetric.** Clearance over a
crossed feature requires that feature's height. But the two sides are not
peers: a street crossing a railway must clear the railway, and the railway must
not notice. A symmetric constraint pool has to pick a winner from relative
weights, and weights are a lie that mostly works — enough junior demand will
eventually move a senior feature. The asymmetry is real, it is knowable from
class, and it must be structural.

**D5. Multi-scale coherence.** At z8 a viaduct is a stroked line; at z16 it is
a deck with portal faces. Every feature must simplify gracefully while its
*position* stays fixed, and while its reconciliation with the per-zoom terrain
mesh still holds.

---

## 4. The model

### 4.1 Three orderings, kept apart

Three distinct questions are routinely collapsed into one. Keeping them apart
is the core of the design.

| Ordering | Question | Scope | Source |
|----------|----------|-------|--------|
| **Stacking** | Which feature is above which, here? | Local, per crossing | Geometry + level ordinals |
| **Authority** | Which feature yields when they disagree? | Global, per (modality, class) | The class taxonomy (§2.2) |
| **Datum** | Which is solved first, so the other can read it? | Global, per stratum | Authority, made mechanical |

**Stacking and authority are independent axes.** A road bridge over a railway
is *above* the railway and *junior* to it: the road moves, the railway does
not. Reverse the stacking and the authority is unchanged:

| Case | Stacking | Who moves | Result |
|------|----------|-----------|--------|
| Road over rail | road above | road | Road climbs to clearance; rail holds |
| Road under rail | road below | road | Road dips into a trough; rail holds |
| Rail over road | rail above | road | Road dips; rail holds |
| Rail under road | rail below | road | Road climbs; rail holds |

In all four the railway never moves. **Authority chooses the mover; stacking
chooses the direction.** Collapsing them produces both failure modes: a railway
lifted to make room for a street, and a street buried because a railway was in
the way.

The corollary makes the whole problem tractable: **once authority is settled, a
crossing constraint has one free side.** It stops being a negotiation between
two variables and becomes a bound on one variable against a known constant.

### 4.2 The strata

A **stratum** is a set of features that share an authority level and may be
mutually coupled. Within a stratum, features are solved jointly. Between
strata, the dependency is one-directional: a stratum reads its seniors as
immovable constants and never writes to them.

The boundary test is not modality but **right-of-way independence**: does this
feature's alignment exist independently of the stratum below it? A mainline
railway on its own formation does. A tram in a street does not — it lies on the
carriageway and is draped on it.

| | Stratum | Members | Authority | Publishes |
|---|---------|---------|-----------|-----------|
| **H** | Hydrology | Still bodies, watercourses | Absolute | Water datum, freeboard |
| **R** | Independent rail | Mainline, narrow gauge, funicular, metro on own formation | Senior to roads | Rail profiles, rail earthworks |
| **S** | Street network | All drivable roads, all classes | Negotiating layer | Road profiles, road earthworks |
| **D** | Draped features | Paths, tracks, steps, cycleways, street-running rail | None | Nothing |
| **B** | Buildings | Footprints | None | Nothing |

**H — Hydrology.** Water level is set by gravity and catchment; no bridge
changes a river's level. Authority is absolute, but the *data* is thin (no
depths, widths, or gradients), so the vertical model is deliberately modest:

- Still bodies are **flat**, at a level read from the DEM around their
  shoreline.
- Watercourses are **monotone descending** along flow.
- **Water constrains; it does not excavate.** The DEM already images real
  watercourses at their level. Carving line rivers into terrain manufactures
  trenches — a defect class of its own. Only still bodies reshape the ground,
  and only by flattening.

**R — Independent rail.** Surveyed alignment, standard-bound, and decisively
**the reason its cuttings and embankments exist**: the terrain is a response to
the railway, not the other way round, so the alignment must be solved before
the ground it created is available to anyone else. Rail is also where
class-specific constraint *shape* matters, not merely parameter values — a
funicular's constraint is *constant grade*, not *bounded grade* (§9).

**S — Street network.** The negotiating layer, and the only stratum that is
genuinely a joint solve: interchanges, junction clusters and dual carriageways
are mutually coupled and cannot be stratified against each other. Internally
the classes form a stiffness ladder (motorway stiff, service soft) expressed as
*mass* within one system, not as separate strata.

**D — Draped features.** Zero authority and **no solve at all**: they sample
the finished ground. Where one carries a genuine structure, that structure is a
local deck fitted to the finished world and constrains nothing upstream. This
is 46.9 % of the road network (§2.2), so the discipline is load-bearing: any
loophole admitting a D feature into a solve is a loophole through which half
the network can perturb the other half. In particular, **carrying a structure
span is not a promotion**.

A fitted deck has no anchors and no grade ceiling, so its whole vertical answer
rests on the ground read at the two ends of the annotated span — which §2.1
says is exactly where the data is least trustworthy. Against a near-vertical
wall the two errors compound: two metres of plan disagreement between the
annotation and the DEM is a dozen metres of height, and the chord starts part
way down the gorge it is meant to cross. One constraint answers it, and it is
about *where a structure may begin* rather than how high it is: **a path cannot
descend a cliff.** Where the ground immediately outside an abutment climbs
faster than the class walks, that abutment did not land on a bank, and it walks
outward along the path's own line to ground that can carry it — stopping at the
bank, or at the height of the span's other abutment, whichever comes first. The
cap is what keeps this a fitting rule rather than a solve: the correction can
only ever make a deck less steep, and the new abutment is a point on the ground,
so the deck still meets the ground at both ends and the approach drapes up to it
with no step.

**B — Buildings.** Founded on the finished ground; no authority over any
network.

### 4.3 Datums and the accumulating ground

Each stratum publishes a **datum**: a queryable field, not a data structure.
The junior stratum asks questions; it cannot reach inside and it cannot write.

```
height_at(p)         -> Option<height>   where this stratum defines a surface
clearance_over(p, k) -> height           what a crosser of kind k must leave
ground(p)            -> height           the engineered ground through this stratum
```

**The ground accumulates through the stack.** This is the load-bearing
structural idea, and it generalises "one ground function" from a convention
into an ordering:

```
ground₀     = conditioned DEM
groundₙ₊₁   = groundₙ  ⊕  stratum n's earthworks
```

Each stratum imprints on the ground its senior published and publishes the
result. A road cutting is carved into a ground that already contains the rail
embankment; a building is founded on a ground that already contains both. There
is never a moment when two stages hold different opinions about the ground,
because there is only ever one ground and it only moves forward.

### 4.4 The solve inside a stratum

Within a stratum the problem is a single constrained fit over one graph:

- **One variable per shared node.** Where alignments meet, they *share* the
  variable. Continuity becomes a property of the degree-of-freedom layout
  rather than a constraint that can fail, so no step is representable.
- **A constraint hierarchy.** Conflict spends the softest thing available,
  never the hardest:

  | Level | Constraints | On conflict |
  |-------|-------------|-------------|
  | **Required** | Continuity, vertical order, monotone water, contact | Never violated |
  | **Strong** | Grade ceiling, clearance minimum | Honoured, or absorbed by penalised slack |
  | **Soft** | Terrain adherence, smoothness, deviation budget | Yields first |

- **Mass decides who moves *within* a stratum.** Terrain-pinned at-grade nodes
  are heavy; lifted and structure nodes are light. A correction distributes by
  inverse mass, so approaches bend to meet decks and decks hold their line.

**A global solve stays local.** The terrain-adherence term puts a mass on every
node, which turns the bare smoothness Laplacian into a *screened* one: a
disturbance's influence decays exponentially, with a length scale set by the
ratio of smoothness weight to terrain weight. A junction's correction fades over
a few hundred metres instead of propagating across a whole component. This is
what makes a globally solved model safe to cut into tiles (I5), and what bounds
the number of sweeps an iterative solver needs. The solve is global in
*statement* and local in *effect*.

Two structures follow:

**Cross-stratum constraints are boundary conditions, not couplings.** A senior
feature enters a junior stratum's system as a *constant*, with no variable of
its own. This is the mechanical statement of authority, and it means the design
needs no second solver:

> **A stratum boundary is the limit of the mass ladder — infinite mass.**

One solver, run over a partition.

**Junctions and structures are entities with their own degrees of freedom.**
"A junction is flat and regular" is not expressible as a set of independent
shared node heights. A junction cluster is modelled as a **plane** — one height
and two slopes — with the incident legs attaching to it, so flatness cannot be
violated rather than merely being checked. The same reasoning gives a structure
entity one grade line that its parallel carriageways read, which is what makes
a dual carriageway on one viaduct come out as one structure (S8).

### 4.5 Structures are consequences, not inputs

> **Solve heights subject to constraints, then synthesize the structure the
> result implies.**

A deck exists where the solved surface departs the ground beyond a threshold. A
bore exists where it runs below. Portals are where it crosses. The data's
level, bridge and tunnel annotations are **priors on the constraint** — a hint
that a clearance exists here and an ordering for it — never commands to build
geometry.

This inversion removes an entire class of contradiction. When structures are
inputs, a stage that later decides a declared bridge is not real leaves the
clearance demand it justified still standing in the system, and that orphaned
demand asks at-grade surface to climb into the air for a deck that no longer
exists. When structures are outputs, such a state is **unrepresentable**: there
is no "crossing whose bridge was deleted", because bridges were never inputs.
It is also what makes the model robust to the annotation noise of §2.1, which
is the same problem wearing a different hat.

Two consequences are normative:

- **A crossing is derived, never stored across a mutation.** Anything that
  changes a feature's geometry or span structure invalidates every crossing
  derived from it. Crossings are re-derived against published datums at the
  start of each stratum's solve (§5).
- **A same-level crossing is information, not an absence.** A road meeting a
  railway at grade is a level crossing: the two surfaces must *share* a height
  there. That is an equality constraint — stronger and more useful than the
  inequality a grade separation gets. Discarding it throws away the one place
  the two strata are known to touch.

### 4.6 Degradation and back-edges

Real cases run against the ladder. The design names them rather than
deadlocking.

| Case | Resolution |
|------|-----------|
| **Aqueduct or canal on a bridge** | The structure belongs to R or S; the water *rides* it as a carried attribute. It never enters the hydrology datum, because it is not gravity-defined terrain water. |
| **Rail bridge abutting a road embankment** | A genuine back-edge. Admitted only as a *soft* constraint inside the junior stratum: the road yields, the rail never learns of it. |
| **Jointly designed grade separation** | The joint optimum is forfeited by construction. Accepted: the error is bounded by the junior feature's own deviation budget, whereas getting authority wrong is unbounded. |
| **Unclassifiable feature** | Assigned to the most junior plausible stratum. A misclassification that costs authority is recoverable; one that grants it is not. |

The rule in one line: **a back-edge never inverts the stratum order.** It is
either demoted to a soft constraint within the junior stratum, or the feature
is reclassified into the senior one. Both are declarable, and both degrade to
something plain.

---

## 5. The pipeline

Tiling is the last and dumbest step. Stratification interleaves solving and
ground derivation, because each stratum's earthworks are part of the ground the
next stratum reads.

1. **Assemble.** Join segments into corridors, resolve linear references, group
   buildings, index water. Classify every feature into a stratum by (modality,
   class) and right-of-way independence (§4.2). Resolve entities: structure
   runs, junction clusters, parallel carriageways. **No crossings are stored** —
   they are derived per stratum in step 3.

2. **Condition the terrain.** Produce `ground₀`: the DEM with narrow notches
   filled and narrow convex bumps shaved, both bounded in span and depth, so
   DEM noise never enters a profile while genuine relief always does.

3. **For each stratum, in authority order (H → R → S):**
   1. *Derive crossings* against the datums already published — never against
      an earlier draft of the model.
   2. *Solve* the stratum's constraint graph (§4.4), with senior datums as
      constants.
   3. *Imprint* the earthworks the solution implies onto `groundₙ`, producing
      `groundₙ₊₁`.
   4. *Publish* the stratum datum.

4. **Synthesize geometry.** Parameterized generators reading solved heights and
   the final ground, adding no new inference: surfaces on their profiles, decks
   and bores where §4.5 says a structure exists, portal faces at the true
   emergence, buildings with foundations and roofs. Draped features (D) and
   buildings (B) are resolved here against the finished ground — they never
   solved. Each generator has LOD variants that keep position fixed while
   shedding detail (D5).

5. **Tile.** Clip, quantize, encode. Because every height is a function of the
   global model, adjacent tiles and successive zooms agree by construction.

Three principles run through all stages:

- **Priors as parameters.** Everything the data does not say enters as named,
  (modality, class)-keyed parameters in one place (§9).
- **A degradation ladder per feature.** Every generator has a defined fallback
  chain (full structure → bare deck → draped line) triggered by data quality.
- **Stage-boundary testability.** Each stage and each stratum emits a plain,
  inspectable artifact validated against §6 and §7 without running what follows.

---

## 6. The canonical situations

A generator is adequate when it handles all of these. Each is a test scenario.

| # | Situation | What determines the height | What it stresses | Mechanism |
|---|-----------|---------------------------|------------------|-----------|
| S1 | **Valley viaduct**: at-grade anchors on both flanks, ravine below | Grade line between anchors | Profile reconstruction, piers, multi-segment structure entities | Structure entity (§4.4) |
| S2 | **Saddle bridge**: short span between two flanks at similar height | Grade line ≈ flat chord | S1's degenerate case; deck ≈ level | Structure entity (§4.4) |
| S3 | **River bridge on flat ground** | Freeboard over the water surface, not the terrain | Feature clearance (water); approach ramps must rise from flat ground (D3) | H datum (§4.2) |
| S4 | **Overpass / interchange on flat ground**, possibly stacked (+1, +2) | Clearance over the crossed road(s); level ordinals resolved to heights | Crossing detection, network constraints (D4), embankments (D3) | Stacking DAG within S (§4.1) |
| S5 | **Mountain tunnel** | Road runs under the surface; portals at the true emergence, not the annotation edge | Annotation mistrust, portal placement, terrain holes (D3) | Structure as consequence (§4.5) |
| S6 | **Urban underpass / cut-and-cover** | Depression below grade with retaining walls, crossing feature above at grade | The flat-ground tunnel case terrain alone cannot express | Imprint (§4.3) |
| S7 | **Bridge directly into tunnel** (portal at the abutment) | Both at once; the deck must meet the portal face exactly | Structure-to-structure continuity | Shared node variable (§4.4) |
| S8 | **Dual carriageway on one structure** | One shared grade line, one (or two abutting) decks | Entity resolution across parallel segments | Structure entity (§4.4) |
| S9 | **At-grade mountain road** (hairpins, 10 %+ slopes) | The terrain itself; no structure | Knowing when to do nothing; grade limits must not "fix" a road that genuinely climbs | Soft-only constraints (§4.4) |
| S10 | **Annotation noise**: spans that overlap, leave slivers, extend past the physical structure, or end before the road reaches the ground | none | Robustness; graceful degradation; solved structure ends | Structure as consequence (§4.5) |
| S11 | **Building on a steep slope** | Footprint meets the ground on every side: downhill foundation or cut platform | Building-ground reconciliation (D3), per-LOD terrain agreement | B against final ground (§4.2) |
| S12 | **Dense old town with courtyards** at several zooms | Footprint interior rings; roof forms from sparse tags | Roof synthesis, courtyard meshing, LOD aggregation (D5) | Synthesis (§5 step 4) |
| S13 | **Building beside a road cut or embankment** | The *engineered* ground, not the natural terrain | Cross-class ground agreement (D4) | Ground accumulation (§4.3) |
| S14 | **Lakefront**: flat water, shoreline, roads and buildings at the edge | Water level as a constraint on the ground and its neighbours | Water surfaces, shoreline continuity | H imprint, then S (§4.3) |
| S15 | **Level crossing**: road meets rail at grade | Neither — the two surfaces coincide | The equality case of vertical order; the one place two strata are known to touch | Equality constraint (§4.5) |
| S16 | **Street-running tram** | The carriageway it lies on | Right-of-way classification: a rail modality with no authority | D stratum (§4.2) |
| S17 | **Railway over a road** | The rail alignment; the road ducks under it | Authority independent of stacking | Authority ⟂ stacking (§4.1) |
| S18 | **Rack railway on a 45 % flank** | Its own steep ceiling, held tight to the ground | Per-class constraint shape; a senior datum that must not float | Priors (§9), datum float check (§8) |
| S19 | **Aqueduct on a viaduct** | The structure carries the water | Back-edge handling: a senior modality carried by a junior structure | Back-edge rule (§4.6) |

---

## 7. Invariants

These define correctness for the rendered scene. Each is a predicate over a
stated population, and each has a check in §8.

**I1 — One ground function.** At any plan position there is exactly one
engineered ground height, and every consumer reads it. No generator samples the
raw DEM outside terrain conditioning (§5 step 2).

**I2 — Surface continuity.** Along every path through a network the drawn
surface is C0-continuous and grade-plausible: zero step at shared nodes,
abutments, portals, segment joins, tile seams and LOD switches; and
|Δh| ⁄ Δs within the (modality, class) ceiling along every drawn centerline.

**I3 — Vertical order with plausible clearance.** Wherever two features cross:
if the crossing is ordered, `upper − lower ≥ clearance(kind)`; if it is at
grade, `upper = lower`. Level ordinals give an ordering, never heights.

**I4 — Support and contact.** Nothing floats and nothing is buried by accident:
decks end on abutments that touch the ground, supports reach the ground,
buildings meet the ground on every side, at-grade surfaces lie on the rendered
terrain of every zoom, portal mouths sit exactly on the surface, lakes are flat
and watercourses descend.

**I5 — Determinism across cuts.** Any two tiles and any two zooms derive
identical heights for shared geometry. All heights are functions of the global
model only, never of the tile window.

**I6 — Graceful degradation.** Annotation noise, missing tags and DEM outliers
may cost detail (a structure drawn as a plain draped line) but never produce
spectacle (a deck diving into a ravine, a floating slab, a staircase at a tile
seam).

**I7 — Datum monotonicity (authority).** A feature's solved height is a
function of its own stratum and strata senior to it, and of nothing else.
Equivalently: **deleting every junior feature changes no senior height, bit for
bit.**

**I8 — Ground monotonicity.** `groundₙ₊₁` differs from `groundₙ` only inside
stratum *n*'s declared footprints, and each stratum's imprint is applied
exactly once.

I7 and I8 are *structural* claims: they are established by construction and
falsifiable by a single perturbation experiment, not sampled by a metric.

---

## 8. Verification

Per `docs/VERIFICATION.md`: write the check before the fix, and prefer a
measurement to an impression. A defect found in a render is not fixed until a
check exists that would have found it.

| Invariant | Check | Population | Falsified by |
|-----------|-------|------------|--------------|
| I1 | `ground.single_source` | Every ground read in the emitted scene | Any consumer whose height disagrees with the published ground |
| I2 | `seam.*`, `slope.road_grade` | Shared nodes; consecutive centerline vertices | A non-zero step; a grade past the class ceiling |
| I3 | `clearance.*`, `order.*` | Every derived crossing | Gap below `clearance(kind)`; inverted order |
| I3 (at grade) | `contact.level_crossing` | Every same-level crossing | Road and rail surfaces not coincident |
| I4 | `contact.*`, `clearance.deck_over_ground` | Structure ends, supports, building bases, portal mouths | A gap or an intersection outside the contact band |
| I4 (fitted decks) | `contact.deck_seat` | The lower abutment of every deck fitted to the ground (§4.2, D) | A deck starting below the wall beside it — a footbridge beginning in the riverbed it crosses |
| I5 | `lod.structure_drift`, `seam.*` | Geometry shared between tiles and zooms | Any height difference |
| I6 | `slope.terrain_face`, `slope.carriageway_face`, `slope.terrain_tearing`, plus degradation-ladder assertions | Terrain and asphalt triangles; features with degraded input | A manufactured retaining wall, a torn surface, or a feature that produced spectacle instead of falling a rung |
| **I7** | **`authority.inversion`** | Every senior datum | **Re-solve with a junior stratum deleted; any senior height that changed** |
| I7 | `datum.float` | Every senior node | Height beyond its class deviation budget from its terrain reference |
| I8 | `ground.footprint` | Every ground sample | A change outside the imprinting stratum's declared footprints |
| §4.5 | `crossing.orphan` | Every clearance demand | A demand with no solved feature on both sides — must be structurally zero |

Three notes on what makes these strong:

- **`authority.inversion` is a proof, not a sample.** It re-runs the model with
  a stratum removed and compares bit patterns. It cannot be passed by luck, and
  it is the only check that verifies the design rather than the output.
- **`datum.float` catches errors at their source.** A senior feature drifting
  from its terrain is the *cause* of downstream clearance errors; measuring it
  directly beats measuring the three-stage-downstream symptom.
- **`crossing.orphan` must read zero by construction.** A non-zero count means
  §4.5 has been violated somewhere, and is a design failure rather than a
  quality regression.

Every check states its population and its coverage limits explicitly. A metric
that silently samples a subset reads as "covered everything" when it did not.

---

## 9. Priors

Everything the data does not say enters as named parameters in one table, keyed
by **(modality, class)**. A single flat road-class enum cannot express the
taxonomy of §2.2 and is the root of the authority failures of D4.

Per entry:

| Field | Meaning |
|-------|---------|
| `grade_shape` | `bounded(g)`, `constant(g)`, or `curvature_limited(g, R)` |
| `deviation_m` | How far the profile may leave its conditioned terrain reference |
| `node_spacing_m` | Profile resolution |
| `clearance_over_m` | What a feature crossing *over* this must leave |
| `clearance_under_m` | What this must leave when it crosses over something |
| `min_structure_m` | Shortest plausible real structure of this class |
| `width_m` | Physical cross-section |
| `stratum` | H, R, S, D or B |

Representative values (calibrated separately; the shapes are the design point):

| Modality/class | `grade_shape` | Stratum |
|----------------|---------------|---------|
| rail / `standard_gauge` | `curvature_limited(0.03, R)` | R |
| rail / `funicular` | `constant(g)` — one gradient end to end | R |
| rail / `tram` | *draped* — no profile | D |
| road / `motorway` | `bounded(0.06)` | S |
| road / `residential` | `bounded(g)`, wide deviation — follows the hill | S |
| road / `footway` | *draped* — no profile | D |

The funicular is the reason `grade_shape` is a shape and not a number: a
constant-gradient alignment cannot be expressed as a ceiling, and a parameter
that pretends otherwise is a lie the solver will act on.

---

## 10. Open questions

- **Classification confidence.** Stratification makes a wrong class *more*
  consequential: a feature in the wrong stratum has the wrong authority. The
  extract carries 5,194 rail segments classed `unknown`. §4.6's rule (assign to
  the most junior plausible stratum) is the safe default, but confidence
  deserves its own degradation rung.
- **The joint optimum for co-designed infrastructure** is forfeited by
  construction (§4.6). Bounded, but real.
- **Intra-S stratification.** Whether motorways deserve authority over service
  roads, or whether the mass ladder within one joint solve is sufficient.
- **DEM conditioning** remains a separate problem: every stratum anchors to a
  terrain reference, and noise there propagates everywhere.
- **Roof and façade synthesis** sits downstream of everything here and is
  largely untouched by it.
