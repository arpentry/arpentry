# The Engineered Ground and Its Meshes

`docs/GENERATION.md` states the vertical problem and prescribes that the
ground be an *output* of generation (D3, invariant 1). This document states
how: the universal road profile that stage 2 solves, the imprint rule by
which stage 3 pulls the natural DEM to those profiles, and the meshing rule
by which the per-zoom terrain meshes preserve the result. It exists because
the three are one mechanism — a road is only as smooth as the ground drawn
under it, and the ground is only correct where the profile is plausible.

The failure mode this design retires: the rendered terrain lattice is far
coarser than an earthwork's footprint (a z14 cell spans ~150 m against an
~18 m bench reach), so a grade-limited cut often moved no mesh vertex and
the road had to be clamped up onto every bump the lattice failed to carve.
The road broke visibly against each one. The fix is not a finer lattice —
it is a mesh that *knows where the benches are*.

## 1. The universal profile

Every drivable road gets a solved elevation profile. There is one vertical
model; "engineered corridor" versus "minor street" is a difference of
*parameters*, not of mechanism.

**The conditioned reference.** A profile anchors to the rendered terrain at
the reference zoom, conditioned symmetrically before anything reads it:

- *Closing* (fill): notches narrower than `NOTCH_SPAN_M` are filled up to
  `NOTCH_FILL_MAX_M` — an at-grade road bridges a stream-cut V-notch on a
  short culvert rather than diving through it.
- *Opening* (shave): convex bumps narrower than `BUMP_SPAN_M` are shaved
  down to `BUMP_SHAVE_MAX_M` — DEM noise (DSM artifacts, upsampling ripple)
  never enters a profile. A run whose shave would exceed the cap is
  reverted: a genuine crest keeps the terrain (the S9 mirror — the road
  climbs a real hill, it does not tunnel through survey noise).

Both operators are bounded in span and depth; beyond the bounds the terrain
is trusted. The *raw* terrain remains available for structural reads (rim
anchoring, portal emergence, deck daylighting), which must see the ground
as it is, not as conditioned.

**Per-class parameters.** Each road class carries a grade ceiling
(`grade_limit` for engineered classes, a bed grade for streets), a
deviation budget (how far the profile may leave the conditioned reference:
generous for a motorway, tight for a lane), and a node spacing (dense for
engineered classes, sparse for the long tail of streets — the cost bound on
profiling the whole network). Engineered-only behaviors — rim anchoring,
infeasible-anchor absorption into structures, deck ramps — stay gated on
the class having a true engineering grade.

**Vertical-curve smoothing.** Engineered profiles round grade breaks only
on nodes lifted off the terrain (draped nodes stay pinned). Street profiles
smooth all nodes, clamped each pass to the class deviation budget of the
conditioned reference — the symmetric low-pass that removes residual
wobble without letting the street float.

**Welds.** Junction continuity holds across the whole network. The
structural weld (raise-only, corridor junctions, clearance-aware) runs
first; the street weld then groups profile endpoints by exact connector
coordinate and pulls them to one height — the engineered corridor's road
height where one passes through, the mean otherwise — with a trust cap
(`BED_WELD_MAX_M`) beyond which the disagreement is treated as a data
contradiction and left alone. Corrections decay into each profile at its
class grade. All deltas are computed against pre-weld heights, so the
result is independent of processing order (invariant 5).

## 2. The imprint

Wherever a profiled road departs the natural ground by more than the
earthwork threshold (`MIN_EARTHWORK_M`), the ground is pulled to the
profile. One rule, cut and fill alike:

- **Flat across the bench**: within the bench half-width (structure
  half-width + shoulder + rendering margin), the ground *is* the profile.
- **Feathered at the batter**: from bench edge to toe, the pull decays
  smoothly over a feather proportional to the depth of cut or height of
  fill (`EARTHWORK_BATTER`).
- **Blended by share at overlaps**: where benches overlap — junction
  approaches, hairpin legs, parallel carriageways — targets blend by
  envelope share, clustered per approach, so overlapping earthworks ramp
  into each other instead of stepping.

Carves (portal cuts, under-deck daylighting) remain separate, winner-take-
all, cut-only notches — a carve is a hole, not a target.

The ground model is a pure function of the global solved model: built once,
queried pointwise, identical from every tile that asks (invariant 5).
Terrain meshing, road draping, surface bands, junction plates, and building
founding all read this one field and nothing else (invariant 1).

## 3. The meshing rule

**What constrains.** For every earthwork run the imprint implies two
polylines per side: the *crest* line at the bench edge and the *toe* line
at the feather's end. These are the contact lines of GENERATION.md stage 3.
At detail zooms (z ≥ z_ref) the terrain mesh is a constrained
triangulation: the regular lattice persists as background points — every
vertex the unconstrained mesh would have, it still has — and the contact
lines enter as constraint edges. Between crest lines the triangulation
cannot place a triangle that straddles the bench, so the drawn ground
holds the bench exactly under every road.

**The z rule.** Constraint vertices carry no height of their own; every
mesh vertex, lattice or constraint, evaluates the one ground function at
its position. Breaklines say where to sample, never what value to find.

**The tile-border contract.** Contact lines are global polylines clipped
per tile. A border crossing lands exactly on the quantized tile edge
(16384 / 49152 on the crossed axis); the neighbor clips the same global
polyline against the same border and derives the identical vertex by
determinism. Interior triangulations of adjacent tiles need not match —
only border vertex positions and heights must, and the client's skirts
close the residual cracks as they do today.

**Normals.** At detail zooms vertex normals are central differences of the
ground function at a fixed metric step — a property of the field, not of
the mesh — so they are continuous across tile borders, flat across a
bench, and creased at a batter regardless of how the triangulation fell.

**The LOD ladder.** Coarser rungs (z < z_ref) keep the plain regular
lattice: at those viewing distances a bench narrower than a cell is not
worth vertices, and roads compensate with the datum lift (§4). The
constrained mesh is a detail-zoom instrument.

**Degradation.** A tile whose constrained triangulation fails — degenerate
constraint after quantization, offset self-intersection the pre-pass did
not resolve — falls back to the plain lattice for that tile (invariant 6:
plain, not wrong). Hairpin insides whose offset radius exceeds the curve
radius drop the inner contact line rather than emit a folded one.

## 4. What roads read

- **At detail zooms** the road reads its profile directly: the terrain
  holds the bench, so profile height and drawn ground agree by
  construction. No clamp, no lattice-crossing chords.
- **At coarser zooms** the road reads that zoom's rendered surface plus a
  clamped datum lift — its solved height expressed relative to the
  reference-zoom ground, never below the drawn surface. This is the
  coarse-LOD rule, not a workaround: the coarse lattice cannot carry a
  bench, so the road hugs the terrain that *is* drawn (invariant 4).
- **On structures** the road rides the deck ramp at every zoom, the same
  heights the deck and bore solids are swept from (invariant 2).
