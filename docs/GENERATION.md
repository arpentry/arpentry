# Procedural World Generation

This document states the problem of generating a coherent 3D scene (terrain,
roads, bridges, tunnels, buildings) from 2D map data and an elevation model,
at the quality of Apple Maps or Mapbox. It names the entities, the data, the
gap between the two, the situations a correct model must handle, the
invariants that define correctness, and the approach they imply: build one
world model on the server, then tile it. It owns the *vertical* model —
heights, structures, the engineered ground; its horizontal companion,
`docs/ROADS.md`, states the road-surface problem (widths, junction areas,
markings) on top of it.

## 1. What is being modeled

The scene is a **built landscape**: natural terrain reshaped by engineering,
with a road network laid over it, structures where the network and the ground
disagree, and buildings founded on it. Four observations drive everything
else:

1. **The ground is the shared substrate.** Every feature class is defined
   relative to the ground: roads lie on it, buildings are founded on it,
   bridges stand above it, tunnels pass under it. Scene coherence reduces
   largely to every generator agreeing on *one* ground surface.

2. **The ground itself is partly man-made.** Embankments, cuttings, portals,
   retaining walls, and building platforms all reshape the terrain. A
   generator that treats the DEM as read-only cannot express them; the ground
   must be an *output* of generation, not only an input.

3. **A road holds an engineered grade.** A motorway is built to ≤ 6 %, a
   railway to ≤ 2–4 %, a residential street to whatever hill it sits on. The
   road's elevation profile stays gentle even where the terrain under it is
   steep.

4. **A structure exists exactly where the road's grade and something else
   disagree.** That "something else" is one of two things, and each calls for
   different information:

   - **Terrain.** A valley that dips below the road's grade calls for a
     viaduct; a hill that rises above it calls for a tunnel. The road profile
     against the terrain sets the structure's vertical position: the deck
     height is the grade line, and the portals sit where the road pierces the
     surface.

   - **Another feature.** A road crosses another road, a railway, a canal, or
     a navigable river, often over flat ground. The terrain says nothing:
     clearance over the crossed feature sets the height (about 5 m over a
     road, more over rail and shipping lanes), and the ground must be
     reshaped (approach embankments, retaining walls) to lift the road there.
     The same split holds underground: a mountain tunnel reconciles the road
     with the terrain; an urban underpass reconciles it with a crossing
     feature, sinking the road below grade between retaining walls.

Buildings only look simpler. A footprint on sloped ground has no single base
elevation: the structure must meet the ground on every side, with a
foundation reaching downhill or a platform cut into the slope. The generator
must synthesize roof forms from sparse attributes. And at every zoom the
buildings must sit on the *rendered* terrain, which differs per LOD.

## 2. The data and what it does not say

**Vector map data (Overture / OSM).**

- *Roads*: 2D centerlines with class attributes and linearly referenced level
  annotations: fractional spans tagged with a small integer level (positive =
  elevated, negative = below ground, 0 = grade). Lane counts, carriageway
  widths, and surface materials ride the same linear referencing; they bear
  on the road's *horizontal* extent and are treated in `docs/ROADS.md` §2.
  Six caveats:
  - **Levels are ordinals, not heights.** `level = 1` means "above whatever
    is level 0 here"; `level = -5` does not mean 5 units deep. Crossing
    bridges at +1 and +2 encode an *ordering* that must be resolved into
    heights.
  - **No elevations anywhere.** Not on nodes, not on structures.
  - **Span boundaries are not registered to the terrain.** Where a tunnel
    annotation ends is where a mapper split it, not where the road emerges
    from the hillside. Annotation edges of adjacent structures routinely miss
    each other by metres, leaving phantom at-grade slivers.
  - **A structure is not an entity.** One physical viaduct may be split
    across many segments, each with its own annotation; one segment may run
    at grade, climb a bridge, dive into a tunnel, and return to grade.
    Nothing links the five bridge spans of a 2 km viaduct into one structure
    with one grade line; that connectivity must come from the graph.
  - **Crossings are implicit.** No link connects an overpass to the road it
    crosses; the crossing must be found geometrically (2D intersection plus
    level order).
  - **Dual carriageways are separate segments** that each claim to be a
    bridge, though the physical viaduct is one structure.
- *Buildings*: 2D footprints (with interior rings) and sparse attributes:
  height or floor count for some, roof shape for fewer, nothing for many. No
  foundation or terrace geometry, no relation to the ground.
- *Water*: polygons and centerlines, no surface elevations. A lake must come
  out flat; a river must descend monotonically; neither is stated.

**The DEM.** A ground-surface raster whose resolution ranges from 30 m
(global) to 0.5 m (national lidar):

- A DSM-flavored source may already *contain* the bridges, buildings, and
  embankments the generator is trying to synthesize; a DTM under a viaduct
  shows the ravine floor. Either way, the DEM near a structure is the least
  trustworthy place to read the road's own height; at-grade stretches, not
  structure spans, must anchor any reconstructed profile.
- What the client renders is not the DEM but a decimated terrain mesh per
  zoom. "Sits on the ground" must hold against the *rendered* mesh at each
  zoom, or roads swim through hillsides and buildings hover. Ground
  reconciliation is therefore per-LOD even when the underlying model is not.

**What no data source provides.** Pier positions, deck cross-sections,
structure type (girder, arch, cable-stayed), abutments, portal architecture,
retaining walls, foundation and terrace geometry, roof details, actual built
clearances. All of it must be synthesized from priors (feature class, span
length, height over ground) if it is to appear at all. Apple and Google do
the same outside their hand-modeled landmarks: parameterized generic geometry
that convinces by getting the *vertical relationships and contacts* right,
not by architectural detail.

## 3. Why this is hard

**D1. The vertical geometry is under-determined.** The data gives topology
(what is above what, roughly where things start and end); the render needs
geometry (heights everywhere). The missing information must be *inferred*
from the terrain plus engineering priors: grade ceilings, clearance minimums,
flat lakes, buildings that meet the ground. The inference is a constrained
fitting problem whose constraints ignore feature boundaries: a deck height
depends on anchors hundreds of metres away, on the road it crosses, and on
the neighbouring segments of the same viaduct.

**D2. The model is global, the tiles are local.** A viaduct's grade line is a
function of kilometres of context; a tile is a small window. Whatever a tile
contains must agree exactly with its neighbours (no seam steps) and with
other zooms of the same area (no popping). Any scheme that lets a tile infer
heights from the data visible in its own window will fit a different answer
than its neighbour fit: a viaduct fragment clipped at a tile edge carries
almost none of the context that determines its height. This is the central
argument for resolving the model *once, globally, before tiling*.

**D3. The ground is an output.** Embankments, cuttings, portals, and building
platforms reshape the terrain. Draping over an untouched DEM can fake some of
this with occlusion and depth bias, but not all of it: an overpass approach
without an embankment floats over flat grass; an underpass without a cut
vanishes into the ground plane; a building on a steep slope gapes on the
downhill side. Reaching the quality bar requires the terrain meshes to
respond to the solved feature model, a generation-time coupling between
layers that must be faced as an architectural decision.

**D4. Features interact.** Clearance over a crossed road requires that road's
profile at the crossing. Two structures computed independently can collide.
An interchange is a small constraint network: ramps, levels, shared columns.
A building platform must not undercut the road beside it. No per-feature pass
can solve this; it takes a pass over the assembled scene with the
interactions made explicit.

**D5. Multi-scale coherence.** At z8 a viaduct is a stroked line and a city a
texture; at z13 slabs and extruded blocks; at z16 piers, railings, portal
faces, roof forms — and the road itself changes representation, from an
SDF-stroked centerline at coarse zooms to a meshed surface with painted
markings at detail zooms (`docs/ROADS.md`). Every feature must simplify
gracefully across zooms while its *position* stays fixed (a deck must not
change height between LODs; a road must not move or change width when the
stroke hands off to the surface), and while its reconciliation with the
per-zoom terrain mesh still holds.

## 4. The canonical situations

A generator is adequate when it handles all of these; each stresses a
different part of the problem. They are the test scenarios for any design.

| # | Situation | What determines the height | What it stresses |
|---|-----------|---------------------------|------------------|
| S1 | **Valley viaduct**: at-grade anchors on both flanks, ravine below | Grade line between anchors | Profile reconstruction, piers, multi-segment structure entities |
| S2 | **Saddle bridge**: short span between two flanks at similar height | Grade line ≈ flat chord | S1's degenerate case; deck ≈ level |
| S3 | **River bridge on flat ground** | Freeboard over the water surface, not the terrain | Feature clearance (water); approach ramps must rise from flat ground (D3) |
| S4 | **Overpass / interchange on flat ground**, possibly stacked (+1, +2) | Clearance over the crossed road(s); level ordinals resolved to heights | Crossing detection, network constraints (D4), embankments (D3) |
| S5 | **Mountain tunnel** | Road runs under the surface; portals at the true emergence, not the annotation edge | Annotation mistrust, portal placement, terrain holes (D3) |
| S6 | **Urban underpass / cut-and-cover** | Depression below grade with retaining walls, crossing feature above at grade | The flat-ground tunnel case terrain alone cannot express |
| S7 | **Bridge directly into tunnel** (portal at the abutment) | Both at once; the deck must meet the portal face exactly | Structure-to-structure continuity |
| S8 | **Dual carriageway on one structure** | One shared grade line, one (or two abutting) decks | Entity resolution across parallel segments |
| S9 | **At-grade mountain road** (hairpins, 10 %+ slopes) | The terrain itself; no structure | Knowing when to do nothing; grade limits must not "fix" a road that genuinely climbs |
| S10 | **Annotation noise**: spans that overlap, leave slivers, extend past the physical structure, or end before the road reaches the ground (a bridge landing into a gorge wall) | none | Robustness; graceful degradation; solved structure ends |
| S11 | **Building on a steep slope** | Footprint meets the ground on every side: downhill foundation or cut platform | Building-ground reconciliation (D3), per-LOD terrain agreement |
| S12 | **Dense old town with courtyards** at several zooms | Footprint interior rings; roof forms from sparse tags | Roof synthesis, courtyard meshing, LOD aggregation (D5) |
| S13 | **Building beside a road cut or embankment** | The *engineered* ground, not the natural terrain | Cross-class ground agreement (D4) |
| S14 | **Lakefront**: flat water, shoreline, roads and buildings at the edge | Water level as a constraint on the ground and its neighbours | Water surfaces, shoreline continuity |

## 5. Invariants

These properties define correctness for the rendered scene and double as
acceptance criteria for any implementation.

1. **One ground function.** A single authoritative ground surface (the
   engineered terrain) that every generator reads: terrain meshing, road
   draping, building founding, structure contact. No feature consults the raw
   DEM on its own.
2. **Road-surface continuity.** Along every path through the network the road
   surface is C0-continuous and grade-plausible: no steps at abutments,
   portals, segment joins, tile seams, or LOD switches. Equivalently, there
   is one road-surface function, and everything that renders (stroked roads,
   meshed road surfaces, bridge decks, tunnel floors) reads from it.
3. **Vertical order with plausible clearance.** Wherever two features cross,
   the higher is strictly above the lower with a class-appropriate gap. Level
   ordinals give an ordering, never heights.
4. **Support and contact.** Nothing floats and nothing is buried by accident:
   decks end on abutments that touch the ground, piers reach the ground,
   buildings meet the ground on every side, at-grade roads lie on the
   rendered terrain of every zoom, portal mouths sit exactly on the surface,
   lakes are flat and rivers descend.
5. **Determinism across cuts.** Any two tiles, and any two zoom levels,
   derive identical heights for shared geometry. Equivalently, all heights
   are functions of the global model only, never of the tile window.
6. **Graceful degradation.** Annotation noise, missing tags, and DEM outliers
   may cost detail (a structure rendered as a plain draped road, a building
   as a flat extrusion) but never produce spectacle (a deck diving into a
   ravine, a floating box, a staircase at a tile seam).

## 6. The approach

The difficulties and invariants force a factoring: every height must be a
function of a globally solved model (D2, invariant 5), the ground must be
derived before anything anchors on it (D3, invariant 1), and interactions
must be resolved before any geometry is emitted (D4). That yields a
five-stage pipeline in which tiling is the *last* and dumbest step:

1. **Assemble.** Build the global scene model from the sources: join road
   segments into corridors, resolve linear references, merge consecutive
   structure spans into structure *entities* (S1, S8), detect crossings
   geometrically and attach the level ordering (S4), group buildings, index
   water bodies. Output: a scene graph whose interactions are explicit.

2. **Solve the vertical model.** One constrained fit over the scene graph:
   terrain anchors at-grade road spans; grade ceilings per class; clearance
   inequalities at crossings and over water; continuity at junctions;
   deviation budgets against the ground; flat lakes and monotone rivers; a
   base elevation per building. *Every drivable road* gets a solved profile —
   engineered classes hold their grade ceiling, minor streets hold a
   per-class bed grade within a tight deviation budget; there is no second,
   weaker vertical model for the unclaimed network. The terrain reference a
   profile anchors to is *conditioned symmetrically*: narrow notches are
   filled and narrow convex bumps are shaved, both bounded in span and depth,
   so DEM noise never enters a profile while genuine relief always does.
   Junction welds hold C0 continuity across the whole network, not only
   where corridors meet. Output: an elevation profile for every drivable
   road, a level for every water body, and a founding plan for every
   building, all independent of zoom and tile.

3. **Derive the engineered ground.** Apply the earthworks the solved model
   implies to the natural DEM: embankment and cutting footprints along roads,
   portal holes, underpass troughs with retaining-wall breaklines, building
   platforms, water beds. The imprint rule is uniform: wherever a profiled
   road departs the natural ground beyond the earthwork threshold, the
   ground is pulled to the profile — flat across the bench, feathered at the
   batter, blended by share where benches overlap. Output: the single ground
   function of invariant 1. The per-zoom terrain meshes are decimated from
   it, under the constraint that decimation preserves the contact lines
   (road edges, building bases, shorelines) each zoom needs: at detail zooms
   the mesh is a constrained triangulation whose constraints are the bench
   contact lines, so the drawn ground holds the bench exactly under every
   road; coarser rungs keep the regular lattice, and roads carry a per-zoom
   datum lift instead (`docs/GROUND.md`).

4. **Synthesize geometry.** Parameterized generators per feature class, all
   reading solved heights and the engineered ground, adding no new inference:
   roads on the profile (stroked centerlines at coarse zooms, meshed
   surfaces with painted markings at detail zooms — `docs/ROADS.md`); bridge
   decks with piers and abutments; tunnel bores with portal faces at the
   true emergence; buildings with foundations, roof forms, and courtyards;
   each with LOD variants that keep position fixed while shedding detail
   (D5).

5. **Tile.** Cut the finished model into tiles per zoom: clip, quantize,
   encode. Because every height is a function of the global model, adjacent
   tiles and successive zooms agree by construction; tiling carries no
   modeling responsibility and parallelizes freely.

Three principles run through all stages:

- **Priors as parameters.** Everything the data does not say (clearances,
  deck thickness, pier spacing, roof pitch, wall heights) enters as named,
  class-keyed parameters in one place: tunable, testable, and honest about
  being priors.
- **A degradation ladder per feature.** Every generator has a defined
  fallback chain (full structure, then bare deck, then draped line) triggered
  by data quality, so bad input degrades to something plain rather than
  something wrong (invariant 6).
- **Stage-boundary testability.** Each stage's output is a plain, inspectable
  artifact (scene graph, solved profiles, ground raster and mesh, geometry
  set) that can be validated against the scenarios (§4) and invariants (§5)
  without running the stages after it.
