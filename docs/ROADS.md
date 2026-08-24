# Road Surface Synthesis

`docs/GENERATION.md` states the *vertical* problem: recovering heights
everywhere from topology and priors. This document states the *horizontal*
one: from 2D centerlines and sparse lane attributes to the paved surface
itself — lane-accurate widths, shared junction areas, and painted markings —
at the quality of Apple Maps' detailed city experience. It names what is
being modeled, the data, the gap between the two, the situations a correct
model must handle, the invariants that define correctness, and the technique
split they imply: **mesh the surface, paint the markings.**

## 1. What is being modeled

The paved surface as an **area**, not a stroke. Four observations drive
everything else:

1. **The silhouette carries the realism.** What makes a detailed city road
   read as real is its outline: corners rounded into fillets, ramps that
   taper as lanes drop, gore wedges where a ramp diverges, legs merging into
   one shared junction area. A constant-width stroke of the centerline can
   express none of these — it has a hard ceiling, however well it is
   antialiased.

2. **The cross-section is the unit of modeling.** At every point along a
   corridor the road is an ordered stack of parallel bands: traffic lanes,
   cycle lanes, parking strips, shoulders, medians, sidewalks. The width is
   the sum of the bands; the markings are the *boundaries between* bands.
   Model the cross-section correctly and both width and markings follow.

3. **Markings are registered to lanes, not drawn freehand.** A dashed lane
   line exists only between two same-direction lanes; a centre line only
   between opposing ones; an arrow sits in a specific lane at a junction
   approach; a stop line spans exactly the approach half of the carriageway.
   Getting markings right is a data-modeling problem; the drawing is the
   easy part.

4. **Junctions are where strokes die.** At an intersection the surface is
   shared: legs join one polygon, corners fillet against *other* roads,
   longitudinal markings stop at a set-back stop line and resume beyond the
   far side. Junction plates conceded this first — a filled area meshed
   precisely because overlapping strokes cannot express it — and P2 increment
   5 generalized the concession to the whole network: at detail zooms the
   drivable surface is *one unioned region*, so a junction is not a special
   object at all, merely the place where several carriageways overlap. What
   survives of the plate is the intersection's *extent*
   (`synth/area.rs`), used to suppress markings, pin the height field, and
   bound the curb-return closing. The *unit* of that extent is the
   intersection, not the connector: the data cuts one place into as many
   connectors as its geometry needs.

## 2. The data and what it does not say

**Overture transportation segments** (derived from OSM) carry, beyond the
`class`/`subclass`/`connectors`/`level_rules` already ingested:

- `width_rules` — linearly referenced carriageway width in metres, from the
  OSM `width`/`est_width` tags.
- `road_surface` — linearly referenced surface material (`paved`,
  `unpaved`, `gravel`, `paving_stones`, `dirt`, …).
- `road_flags` — linearly referenced booleans (`is_bridge`, `is_tunnel`,
  `is_link`, `is_covered`, …).
- `access_restrictions` — the oneway signal: a segment is one-way when a
  restriction denies `access_type = denied` under a `when.heading`
  condition.
- `speed_limits` — linearly referenced posted speeds; a proxy for
  cross-section scale where nothing else is mapped.
- **No lane model.** The schema once carried a `lanes` list; current
  releases do not (dropped pending redesign), and OSM's `lanes` /
  `turn:lanes` tags are lost in the translation. Lane *structure* must be
  inferred (H2) until a lane source exists.
- Sidewalks, crossings, and cycleways are **their own segments** — class
  `footway` with subclass `sidewalk` or `crosswalk`, and class `cycleway`
  (subclass `cycle_crossing`) — not attributes of the road they accompany.
  (OSM's on-carriageway `cycleway:left=lane` / `sidewalk=both` tagging is
  largely lost in the translation; carriageway-attached bands must come
  from priors or wait on richer data.)

All the linearly referenced fields use the same `between: [start, end]`
fractional sub-range structure as `level_rules`; the parsing machinery in
`server/src/levels.rs` generalizes to them directly.

What the Swiss extract actually holds (P0 survey, 2026-07-14; 2.94 M road
segments):

- **Measured width is rare.** `width_rules` covers 0.6–10 % per class —
  1.2 % of motorways, 3.3 % of residential, 3.4 % of primaries. The prior
  is the *primary* source of width, not the fallback. Where mapped, the
  medians validate the priors: primary 7 m, secondary 6 m, residential
  5 m per carriageway.
- **The indirect signals are well covered.** `road_surface` on 91 % of
  primaries (55 % of residentials); oneway-ness on 98 % of motorways,
  79 % of trunks, 29 % of primaries; `speed_limits` on 84 % of primaries
  (medians 30/50/80/90 km/h up the class ladder). Inference must lean on
  these, not on lane data.
- **Partial sub-ranges are real.** 80 % of `road_flags` rules, 17 % of
  `width_rules`, 11 % of `road_surface` rules carry a `between` range —
  the linear-referencing machinery is load-bearing, not decorative.
- **Crossings are dense and usable.** 33.5 k `crosswalk` segments (median
  length 10.7 m — the span of the carriageway they cross, itself a width
  observation of that road), 46 k `sidewalk`, 18 k `cycleway`.

Four caveats:

- **Segments are per-carriageway.** Each direction of a dual carriageway,
  and each ramp, is its own segment (GENERATION.md S8). Widths describe
  one carriageway; nothing may double-count.
- **Crossings are misregistered.** A crosswalk segment is mapped by hand
  across the road it crosses; its endpoints rarely lie exactly on the
  carriageway edges, and its width is a stroke convention, not the painted
  extent.
- **Marking style is almost never mapped.** OSM's `crossing:markings=zebra`
  and friends exist but are rare; Overture drops most of them. Which lines
  are solid or dashed, where stop lines sit, arrow placement — all priors.
- **What no data source provides:** lane counts, edge geometry, fillet
  radii, taper lengths, gore shapes, stop-line setbacks, marking colors.
  As with the vertical model, everything the data does not say enters as
  named, class-keyed parameters in `priors.rs` — tunable, testable, and
  honest about being priors.

## 3. Why this is hard

**H1. The stroke ceiling.** The current rendering strokes a server-emitted
centerline into constant-half-width SDF quads on the client. A stroke cannot
taper, cannot fillet against another road, cannot merge two approaches into
one polygon, cannot cut a gore. No amount of shader work lifts this ceiling;
the representation itself must change from line-with-width to surface.

**H2. Width and lanes are under-determined — in opposite directions.**
With `width_rules` on a few percent of segments and no lane data at all,
the two halves of the cross-section are derived from each other. Width:
`width_rules` when present, else the class prior (today's `half_width_m`),
modulated by what *is* mapped — a link narrows, a one-way carriageway needs
fewer lanes than a two-way one. Lane count: inferred back from the width —
`round(width / lane_width(class))`, floored at one, halved in effect for
two-way roads — because the markings need it even though no source states
it. The chain must always produce a sane answer, and every consumer —
stroke, surface, structure sweep, earthworks, markings — must read the same
one or decks, asphalt, and paint drift apart.

**H3. Junction surfaces are computational geometry.** The shared area is a
union of variable-width leg polygons with filleted mouth corners — under
real-data stress: near-parallel legs, legs shorter than their own trim
radius, duplicated geometry, five-way crossings, roundabouts. Robustness
here is the price of the silhouette.

**H4. Markings need a road-relative parameterization.** Every marking is
naturally expressed in corridor coordinates: `s` (arclength along) and `t`
(signed offset across). Dashes are periodic in `s`; band boundaries are
levels of `t`. The parameterization must survive tiling: a dash pattern that
resets at a tile seam produces a visible stutter. Dash phase must be a
function of *global* corridor arclength, never of the tile window
(GENERATION.md I5).

**H5. Two representations, one road.** At coarse zooms the road stays an
SDF-stroked line; at detail zooms it becomes a surface with paint
(GENERATION.md D5). The switch must move nothing: same centerline, same
total width, same heights (both read the solved profile and the engineered
ground). A road that jumps or fattens at the handoff zoom is worse than
either representation alone.

## 4. The canonical situations

| # | Situation | What determines the surface | What it stresses |
|---|-----------|----------------------------|------------------|
| R1 | **Residential street, no lane data** | Class priors end to end | The fallback path (H2); marking ladder — a quiet street has few or no markings |
| R2 | **Arterial that widens at the junction** | A partial `width_rules` range where mapped, else an approach-widening prior | Linear referencing, width transitions (tapers) |
| R3 | **Dual carriageway** | Two per-carriageway segments, one road | No double-counting; median gap between the surfaces reads as intended |
| R4 | **Motorway lane drop** | Lane count change mid-mainline | Taper priors; edge-line continuity through the taper |
| R5 | **Ramp diverge / merge** | Link segment leaving the mainline | Gore geometry between the diverging edges, chevron hatch, nose rounding |
| R6 | **Signalized four-way intersection** | Union of four legs, filleted | Junction meshing (H3); stop lines set back; longitudinal markings suppressed inside the plate; crosswalks on each leg |
| R7 | **Mid-block crossing** | A `crosswalk` segment across the carriageway | Registration: the zebra must span exactly the carriageway, whatever the mapped stub says |
| R8 | **Cycleway** | Its own segment beside the road | A narrow surface ribbon of its own class; no markings beyond edge treatment |
| R9 | **Sidewalk** | Its own segment | As R8, at pedestrian scale; kerb line where it borders the carriageway |
| R10 | **Roundabout** | Circular junction geometry | Curved plate, fillets against every leg, circulating markings |
| R11 | **Track / unpaved path** | `road_surface` | Knowing when to do nothing: no markings, humbler surface material |
| R12 | **Data noise**: lanes contradicting width, overlapping sub-ranges, a crosswalk floating beside its road | none | Robustness; degradation to a plainer surface, never a broken one |

## 5. Invariants

1. **One cross-section function.** Per corridor, one derivation from data
   and priors to the band stack and total width, read by everything that
   needs a width: the cartographic stroke, the surface mesh, the structure
   sweep, the earthworks. (Today `half_width_m`/`paint_width_m` already
   share one prior; this widens that contract.) The prior is what the class
   asks for; what a stretch of street actually gets is that capped by the
   **room its facades leave** (§6.6 P3 increment 10), which is per station
   and per side — one function still, with an argument the priors alone
   could not supply. A sidewalk is part of that cross-section and not a
   feature of its own: its band takes its shape from the **host centerline**
   over the arc range `assemble::walks` attached it across, never from its
   own mapped polyline, which decides only *where* a sidewalk is, *which
   side*, and *how far out* (`synth::walkway`). The room is spent in order —
   carriageway, then walkway, then verge — so a street with no room for a
   sidewalk simply has none, which is what a narrow street looks like.
7. **A pedestrian way is a surface, not a line.** From
   `WALK_SURFACE_MIN_ZOOM` every footway, path, cycleway and stair is a
   region in the union like a carriageway: its own material, its own hole in
   the drawn ground, its own apron. Two materials, because they are two
   things — a `Walkway` stands a kerb (`KERB_RISE_M`) above the carriageway
   it belongs to, and a `Path` stands on the ground and belongs to nothing.
   Their cartographic strokes are deleted at those zooms for the same reason
   a carriageway's is: the mesh *is* the surface. What keeps a stroke is what
   the walkway model did not draw — a footbridge, a subway, a crosswalk that
   registered against no carriageway, and **any way the model declined to
   band at all**. That last is the test asking what was *built* rather than
   what class a feature is: the seat can run out of room and the ground fit
   can refuse a band on a steep flank, and deleting the stroke on the class
   alone made those ways vanish outright instead of degrading to a line
   (invariant 6 — lost detail, never spectacle). `synth::walkway::bands`
   returns the source of every segment so phase 1 can stamp `walk_banded`,
   which is what `paves_via_walkway` reads. The Territet switchback at
   6.9189,46.4304 is the type specimen: a stair-and-footway zigzag on a flank
   too steep to bench, drawn as a handful of disjoint slabs with nothing
   between them, now continuous — band where one was built, stroke where it
   was not. Granularity is the *feature*, so a long way that is banded along
   part of its length still loses its stroke on the rest.
   And, being a surface, each **benches the ground under it** exactly as a
   carriageway does (docs/GROUND.md §2): the bands are derived once and stage 3
   imprints those same segments in stratum D. The band that draws the surface
   and the bench that holds it up are one cross-section, not two constructions
   of one. That cross-section is allotted out of **three** bounds, not two: the
   kerb it starts at, the facade it stops short of, and — since the band is
   fitted to the senior ground before D benches it — the earthwork its own
   material may plausibly build. Where any of the three leaves less than
   `WALK_MIN_WIDTH_M`, there is no band, which is the same sentence invariant 6
   speaks about a street too narrow for a sidewalk. A path across a 45° flank is
   narrower than a promenade, and past a point it is not a drawn surface at all.
2. **A closed, simple silhouette.** The paved surface has no gaps, no
   slivers, no overlapping fills. Held *by construction* since P2 increment 5:
   the surface is literally one unioned region per level, so there are no two
   objects left to disagree about a boundary.
3. **Markings are functions of the cross-section.** Every painted element
   derives from the band model and the junction topology — a lane line
   exists because two same-direction lanes adjoin, a stop line because an
   approach meets a plate. No marking is placed by eye.
4. **Global phase.** Dash patterns, zebra stripes, and symbol positions are
   functions of global corridor arclength; any two tiles, and any two zoom
   levels, paint identical marks on shared geometry.
5. **Representation agreement.** The stroke (coarse zooms), the surface
   (detail zooms), and the structure decks read the same centerline, the
   same width function, and the same road-surface heights; nothing moves at
   the handoff (GENERATION.md D5).
6. **Graceful degradation.** The ladder runs: full surface with markings →
   plain surface → today's stroke. Data noise (R12) costs paint and
   silhouette detail, never geometry spectacle — no self-intersecting
   fills, no markings wandering off the asphalt.

## 6. The approach

### 6.1 The technique split

The reference quality is reached with two techniques, each doing what it is
good at:

- **The surface is a mesh.** Baked in synth like the decks: every corridor
  centerline buffered by the width function, the results unioned into one
  region, its reflex corners closed into curb returns, triangulated, draped on
  the shared road height field, emitted as `MeshGeometry`. The silhouette —
  fillets, tapers, gores — is real geometry, crisp by construction.
- **The paint is a decal layer.** Markings are thin geometry laid over the
  surface with road-relative coordinates: strips along band boundaries,
  transverse bars, symbol quads. Edges are antialiased in the shader by
  signed distance from the strip axis; dashes and zebra stripes are
  procedural (`fract(s / period)`); symbols (arrows, chevrons) come from a
  multi-channel SDF atlas so they stay sharp at any zoom.

SDF does not go away; it moves up a layer — from drawing the road's
silhouette (the job it cannot do) to drawing the paint (the job it is best
at). The split is by zoom: SDF-stroked roads at coarse zooms, meshed
surfaces with painted markings at detail zooms. Below the handoff zoom the
existing stroke remains, unchanged.

### 6.2 The lane model stays on the server

The client never learns what a lane is. The server assembles the
cross-section, derives the width, buffers the surface, and *bakes the
marking geometry*; the client renders two dumb things — a vertex-colored
mesh it already knows how to draw (the junction plates ride this path
today), and a decal layer with one new pipeline. This pulls the complexity
downward (docs/DESIGN.md): one place understands road anatomy, everything
downstream is geometry.

### 6.3 Pipeline placement

1. **Assemble** ingests `width_rules`, `road_surface`, `road_flags`, and
   the oneway signal from `access_restrictions` (extending the
   transportation column lists and reusing the `level_rules` sub-range
   parsing), and attaches per-corridor *cross-section runs* to the scene
   graph: for each arclength range, the ordered band stack and its
   provenance (measured or prior).
2. **Solve** is untouched — the vertical model is orthogonal.
3. **Ground** reads the width function where it now reads the class
   half-width, so earthwork footprints follow the true carriageway.
4. **Synth** gains the surface baker (corridor polygons unioned into one
   region per level per chunk, absorbing the junction plates entirely) and
   the marking baker
   (band-boundary strips, stop lines, crosswalk boxes, symbol quads). Both
   read solved heights and the engineered ground; markings on structure
   decks read the same road-surface function, so a bridge carries its lane
   lines across (GENERATION.md I2).
5. **Tile** is untouched.

### 6.4 Format and client

- `MeshGeometry` gains optional per-vertex road-relative coordinates —
  `s` (int32, mm of global corridor arclength) and `t` (int16, mm across,
  signed from the centerline) — plus a per-part marking style id. Absent
  arrays mean a plain mesh; existing tiles remain valid. The encoding is
  specified in `docs/FORMAT.md` when implemented.
- The client adds one marking pipeline: depth-biased decal over the road
  surface (the mechanism the road stroke already uses over terrain), SDF
  edge antialiasing from `t`, procedural dash from `s`, and an MSDF symbol
  atlas for arrows and chevrons.
- Marking colors and the surface palette come from the style file, not the
  code — marking conventions are regional (Swiss crosswalks are yellow;
  North American centre lines are yellow) and must stay data.

### 6.5 Priors

New class-keyed parameters in `priors.rs`, in the spirit of the existing
ones: lane width (motorway 3.75 m, primary/secondary 3.25 m, minor
2.75 m), shoulder and edge margins, marking line widths (0.10–0.15 m),
dash period and duty cycle, zebra stripe pitch, stop-line width and
setback, fillet radius by class pair (3–12 m), taper length per lane
dropped, and the marking ladder per class (a motorway paints edge + lane
lines + gore chevrons; a residential street paints little or nothing
unless the data says otherwise). Plus the LOD thresholds, kin to
`STRUCTURE_DETAIL_MIN_ZOOM`: the handoff zoom where the surface mesh
replaces the stroke, and the zoom where paint appears.

### 6.6 Phasing

Each phase lands independently, ends green (`cmake --build build && ctest`,
`cargo test` in `server/`), and is verified with `--screenshot` renders of
the scenario table (§4).

- **P0 — Survey.** Done (2026-07-14, Swiss extract); findings folded into
  §2 and H2. The decisive ones: no lane column exists, measured width is
  rare, surface/oneway/speed are well covered, and crosswalks are keyed
  `footway`/`crosswalk` with lengths that span the crossed carriageway.
- **P1 — The width function.** Done (2026-07-14). `width_rules`,
  `road_surface`, and the `access_restrictions` one-way verdict reduce to
  scalars at decode (`server/src/rules.rs`); `carriageway_width_m` in
  `priors.rs` prefers a plausible mapped width over the class prior; the
  tiles carry the result as `width_m` plus `surface`/`oneway` properties.
  Server-only — the existing stroke picked up variable width with no
  client change. (`road_flags` ingestion waits for its consumer, P2.)
- **P2 — The surface.** The corridor surface baker: buffer, union with
  junction plates, fillet, drape, mesh. Rendered through the existing mesh
  path at detail zooms; the stroke remains below the handoff zoom. This is
  the largest phase and the unlock for everything after it.
  - *Increment 1 — surface bands (done 2026-07-14).* Every drivable
    at-grade road drapes an asphalt band under its paint
    (`server/src/synth/surface.rs`): the baked centerline offset to the
    P1 width plus the structure shoulder, every edge vertex on the shared
    `road::surface_height`, sunk `SURFACE_SINK_M` so plates and decks win
    overlaps; emitted through the junction plates' decode path, zero
    client changes. Small service ways (driveway/parking_aisle/alley)
    narrowed to `SERVICE_WAY_WIDTH_M`. Archive cost ~+60 % at detail
    zooms (`ROAD_SURFACE_MIN_ZOOM` is the knob).
  - *Increment 2 — junction trim (done 2026-07-14, superseded by
    increment 4).* Plate legs span the band edge and surface bands are
    interval-trimmed at each plate's disk, tucked under the mouth so no
    ground sliver can open. Corners curved as quadratic-Bézier curb returns
    with straight-chord fallbacks.
  - *Increment 3 — street beds (done 2026-07-15).* The terrain raster and
    the road network are independent datasets; the engineered ground
    reconciles them (GENERATION.md D3, invariant 1). Every unclaimed
    drivable road benches the ground under its carriageway: earthwork
    edges whose targets are the *natural* ground sampled at the
    centerline — flat across, natural grade along. The rendered lattice
    is far coarser than a street, so at the reference zoom the drape
    (paint, band, markings, street plates) rides the exact bed
    (`Earthworks::target_at`) instead of the tilted in-cell
    interpolation; coarser zooms keep the per-zoom surface, whose corners
    the bench pulls toward the bed wherever the lattice can see it.
  - *Increment 4 — intersection areas (done 2026-07-29).* The unit became
    the **intersection**, not the connector, and the plate became a
    **region** rather than a fan. `synth/area.rs` builds the paved area as
    the union of one rectangle per leg, each anchored at the intersection
    centre; because every rectangle contains that centre the union is
    star-shaped about it, so the boundary is exactly the radial maximum
    `r(θ)` — a closed form with no polygon boolean, no orientation
    predicate, and no configuration needing a fallback (H3, and DESIGN.md's
    define-errors-out-of-existence). The plate ring, the point test
    (`contains`) and the band and marking trims (`clip_chord`, exact
    Liang–Barsky per leg) are three readings of that one region, which is
    what let the mouth-corner walk, the fillet special cases and the
    circular trim disk all be deleted.
    `synth/junction.rs` gained the clustering that precedes it: junctions
    joined by a corridor too short to be a block merge into one
    intersection, shortest-first and refused past `MAX_CLUSTER_M`, so a
    staggered crossroads, a slip-lane nose and a roundabout's ring of
    connectors each plate once instead of piling shards (R6/R10 — a
    roundabout needs no rule of its own). Leg widths now come from the new
    `Corridor::width_m`, the same cross-section the bands read, so a mouth
    and the band landing on it cannot disagree (invariants 1 and 5); a
    non-drivable member (a footway, a crossing) joins an intersection
    without paving any of it, and the plate's styling class is its widest
    drivable member's rather than its highest-standing one's.
  - *Increment 5 — the unioned surface (done 2026-07-30).* The surface
    stopped being a pile of overlapping objects and became **one region**.
    Every carriageway is buffered to its own width and unioned
    (`synth/poly.rs` over `i_overlay`, the only file that names the crate),
    per `(level, grade-separation layer)` and per **chunk** — chunks being z13 tile rects,
    so that every zoom drawing asphalt nests wholly inside one chunk and a
    tile clip never spans a chunk edge (`synth/pavement.rs`). Curb-return
    fillets come from a morphological closing at `CURB_RETURN_M`, restricted
    to the intersection extents: applied globally it would bridge any gap
    under twice that radius and fuse a divided carriageway into one slab.
    `synth/pave_mesh.rs` clips the region to the tile proper, simplifies the
    boundary to the zoom's budget capped at `PAVE_SIMPLIFY_M`, insets it by
    `PAVE_RIM_M` and meshes the interior with the same constrained-Delaunay
    contract as `terrain_cdt`, leaving a rim strip that carries both the
    analytic edge antialiasing and the darker casing tone.

    The interior is triangulated over the **terrain's own lattice**, not from
    the boundary alone. A region meshed from its outline is spanned by
    triangles as long as the road is wide, so the asphalt is a chord across
    whatever the ground does between its two edges, while the terrain beside it
    is sampled every cell: on a cross-slope the two surfaces cross and the
    hillside surfaces through the carriageway in ragged bites — worst exactly
    where the ground stage declined to bench (docs/GROUND.md §2) and the road
    is laid on the natural slope. Sampling the height field at the same lattice
    points the terrain mesh uses makes the two agree at those points by
    construction and leaves only the boundary strip to interpolation. The
    points are global per zoom, so neighbouring tiles derive identical ones,
    and they go in row-major, so the triangulation stays a function of the
    input alone.

    The layer is what keeps a flyover off the road beneath it, and it is not
    optional: Overture's `level` ordinal does not carry grade separation. A
    flyover's bridge span is excluded from the union already, but its
    *approaches* are ordinary at-grade spans at level 0 — and so is the road
    they pass over. Keyed on level alone they merged into one region and the
    mesh ramped continuously between two roads metres apart vertically. The
    layer comes from `solve::crossings::corridor_ranks`, the existing crossing
    DAG, so a corridor that crosses nothing stays at layer 0 and ordinary
    streets still merge at their intersections. The height field partitions the
    same way, or it would blend the two surfaces back together.

    Heights come from a new **road height field** (`synth/height.rs`): one
    continuous function per level, blending the corridors covering a point and
    *overridden* near an intersection by the height the solver made its
    members share — persisted for this out of the constraint graph, which
    previously computed it and threw it away.

    What this deleted is the measure of it: `SURFACE_SINK_M`, `TRIM_TUCK_M`,
    `MIN_PIECE_M`, the leg-offset widening, all of `synth/surface.rs`, the
    plate emission in `synth/junction.rs`, and the client's whole
    no-depth-write `plate_pipeline` — every one of which existed only to
    arbitrate overlaps between objects there is now only one of. At-grade
    asphalt and elevated decks are one depth-writing pass again.
    `synth/area.rs` survives, demoted from the paved plate to the
    intersection's *extent*: marking suppression, the height field's pin, and
    the closing mask.
    Cost is ~1 s on a 61-tile preview (7.9 s before, 9.0 s after). Two things
    make that possible and both are load-bearing: the boundary is simplified
    per zoom (capped at `PAVE_SIMPLIFY_M`, since the generic line budget would
    move a carriageway edge by a fifth of its width), and the height field is
    built **only** for zooms that mesh asphalt — a tile's source query is
    bounded by its own extent, so an ungated z0 tile collected every
    carriageway segment in the extract to draw none of them, which alone cost
    780 s of the run.
  - *Remaining:* the surface is one uniform class (`road_surface`) with a
    darker `road_casing` rim, per the decision that all roads share a colour
    at detail zooms; class distinction now comes from width and paint. Still
    open: `road_flags` ingestion.
- **P3 — Longitudinal paint.** The marking baker and the client decal
  pipeline: centre/lane/edge lines as procedural-dash strips, stop lines,
  zebra crosswalks (procedural stripes registered to R7's carriageway
  span).
  - *Increment 1 — the solid lines (done 2026-07-14).* Markings bake as
    ordinary narrow line features of class `marking`
    (`server/src/synth/markings.rs`), drawn by the existing stroke
    pipeline at physical width over the road paint — no client change,
    one style entry. The ladder paints a centre line on two-way
    primary/secondary/tertiary and edge lines on motorway/trunk, offset
    from the *raw* centerline (an offset of the densified line wobbles),
    trimmed at the junction plates, stubs under `MIN_LINE_M` dropped.
  - *Increment 2 — dashed centre lines, global phase (done 2026-07-14).*
    Marking generation moved to phase 1, cutting dashes from *pre-clip*
    geometry — the whole segment, or a corridor piece at its global span
    boundaries — so the phase anchors to a global arclength origin and
    every tile clips identical copies (H4/invariant 4 without any `s`
    coordinate). The centre line is now the dashed Leitlinie
    (`CENTRE_DASH_M`/`CENTRE_GAP_M`); a dash whose midpoint falls in a
    plate disk is dropped. Markings sort at `MAX_RANK` so they decode
    after the strokes they paint over; `JunctionModel` gained a grid
    index (`near`) so per-segment plate trims scale to full extracts.
  - *Increment 3 — lane dividers (done 2026-07-15).* The H2 inference
    made real: `lane_count = round(width / lane_width(class))`, and a
    one-way carriageway paints dashed dividers at its n−1 interior lane
    boundaries (`priors::has_lane_lines` — motorway/trunk count as
    one-way even untagged, each carriageway is one by construction; plus
    tagged one-way primary/secondary). A 9 m motorway carriageway reads
    as two lanes split by a dashed line inside its solid edges.
  - *Increment 4 — the paint is registered to its own asphalt again
    (done 2026-08-12).* Two independent losses of registration, both in
    the emit stage, both invisible to every check in the scorecard
    because a misregistered marking is still perfectly draped.

    **(a) The offset was projected away.** The baker's arithmetic was
    right — an edge line comes out of `synth::markings` at ±(w/2 −
    0.3 m), a lane divider at its own boundary — and phase 2 threw it
    away: `synth::road::bake` snapped every road vertex onto the
    corridor's smoothed sweep line (so paint would not trace digitising
    wiggle beside its own smooth-swept bridges) by **projecting** it,
    which is the one operation that annihilates a signed offset. Both
    edge lines and every lane divider landed coincident down the middle
    of the carriageway. `Profile::smooth_at` now measures the vertex's
    lateral offset against the raw edge it projects to and re-applies it
    along the smoothed line's own normal: the centerline (offset zero)
    is unmoved, and paint keeps its `t`. This is H4's road-relative
    parameterization made to survive the pipeline, not just the baker.

    **(b) The paint and the asphalt were on different curves.** Even at
    offset zero, at-grade paint was carried onto the *smoothed* sweep
    line while the unioned carriageway is buffered around the corridor's
    **raw** nodes (`synth::junction::carriageway_sources`). The two are
    a median 1.0 m apart on the classes that carry markings, out to the
    `SMOOTH_MAX_DEV_M` clamp — on a 6 m street, a centre line sitting in
    a lane. Paint now rides the line the surface under it was built
    from: the smoothed one on a structure (a deck and a bore are swept
    along it), the raw one at grade, and the smoothed one at zooms below
    `ROAD_SURFACE_MIN_ZOOM`, where no asphalt is drawn and the stroke is
    the road — which is the cartographic reason the snap was written.

    Measured on the Montreux extract, before → after: an edge line's
    near kerb 5.40 m (half a carriageway — the collapse) → **1.30 m**
    against the 1.30 m the cross-section asks for, with p25 and p75 on
    the same value; a centre or lane line's left/right asymmetry 1.35 m
    → **0.05 m** at the median; markings landing off the drawn asphalt
    4.43 % → **0.19 %**. `slope.road_grade` improved with them (6.47 →
    2.94), since paint pulled sideways onto another curve had been
    reading its height from ground it does not lie on, and the rail
    standoff population halved as rail strokes came back onto their own
    ballast. Guarded by `paint.edge_line_inset` and
    `paint.marking_offside` (docs/VERIFICATION.md).
  - *Remaining:* symbols (arrows, chevrons — the MSDF atlas, P4);
    distance fade — sub-texel lines shimmer at grazing angles; stop
    lines; dividers on wide two-way roads (2×2 boulevards).

    *Increment 4b — the crossing is drawn (done 2026-08-24).* R7, and the
    largest single break in the drawn pedestrian network: a `crosswalk`
    segment was neither band (correctly — it is paint on the carriageway)
    nor paint (this increment had never been built), so at the walk zooms a
    crossing was a thin unregistered stroke dissolving into the asphalt, and
    every sidewalk→crossing→sidewalk edge of the mapped graph drew as band /
    nearly nothing / band. Now one registration per crosswalk
    (`synth::walkway::crossings`): the mapped line, extended along its end
    tangents, is sampled against the same corridors and drawn half-widths
    the union buffers; the on-asphalt interval is the kerb-to-kerb **chord**
    and the mapped remainder outside it the **stubs** — one derivation, two
    readers, so paint and kerb meet by construction. The stubs join the walk
    bands (Walkway material, **on the ground** — the end of a crossing is a
    dropped kerb by construction, and a rise would float the band above the
    bench stratum D cuts for a hostless one; `contact.walk_rim` measured
    exactly that 0.12 m float on the first cut) and are fitted, benched and
    unioned like any band. The chord paints a zebra
    ladder (`synth::markings::crossing_bars`, class `crossing`, style-keyed
    colour): stripes longitudinal to traffic at the bar/gap priors, each
    drawn as a *transverse* stroke — the client's round caps reach half the
    stroke width past a line's ends, so chord-wise dashes at the crossing's
    depth grew 1.4 m of cap into every gap and fused into one slab; stroked
    across, the caps round the stripe ends instead, which is how the paint
    wears. A registered crossing's stroke is deleted at the walk zooms
    (`paves_via_walkway` reads the `crossing_drawn` stamp, stripped before
    encoding); an unregistered one — mapped across a path, data noise (R12)
    — keeps the stroke that is all it ever had. A divided carriageway
    registers one chord per roadway and the refuge island is unpainted.
    Registration is against raw centerlines, so it agrees with the drawn
    asphalt to the smoothing displacement (median ~0.5 m at junction
    mouths); the decal bias absorbs it. Guarded by `network.walk_cover` /
    `network.walk_reach` (docs/VERIFICATION.md): crossing-attributed bare
    length on the Villeneuve cut went to zero.

    *Increment 4c — the corner is a sidewalk (done 2026-08-24).*
    `assemble::walks::runs` breaks an attachment where the way turns across
    its host — correctly, a band must not bridge a side street's mouth — so
    the stretch of a sidewalk polyline that wraps a corner attaches to
    nothing, and the path rule then drew it as the wrong feature twice over:
    `Path` material on the ground between two `Walkway` neighbours on a
    kerb, or nothing at all under the 4 m minimum — and a junction's corners
    are exactly where sub-4 m stretches arise, between the two crossing
    connectors of a corner. An unattached gap *pinched between two claimed
    stretches* of the same line, under `WALK_CORNER_MAX_M`, now keeps the
    sidewalk's material and width at any length worth a segment: it is the
    link between two bands, and a link has no minimum worth existing. It
    stands on the ground, not on a kerb — a hostless band's bench targets
    the ground along its own centerline, so a rise is a float above its own
    bench, and the height field ramping the neighbouring kerbs down into the
    corner is what a corner's dropped kerbs are. Longer or unbounded
    stretches stay paths — a tagged sidewalk that wanders off across a park
    is genuinely one.

    *Increment 5 — one centerline (done).* "The at-grade band and the
    deck are not on the same curve" was the long-standing item here, and
    it was four defects rather than one. Measured by
    `verify::checks::abutment`, which reads the handoff directly: the
    approach and the span are cut from a **single shared coordinate**
    (`Corridor::pieces`), so the two ends are one point, zero is the
    correct answer, and there is no prior to negotiate.

    - The sweep line was at the wrong **station**. It was sampled at the
      densifier's *chord* fraction on a centripetal Catmull-Rom, whose
      parameter is not arc length, so `smooth[i]` was not the point of
      the road that `nodes[i]` is — which every consumer assumes by
      index. Median 0.37 m of slide, out to 721 m where vertex spacing
      ran from metres to kilometres; a deck swept there carries the
      height solved for another station and lands its abutment off the
      span. The parameterisation is now inverted per segment.
    - The smoother **cut every corner**. A quadratic in arc length
      reproduces a parabola, not a circular arc, so its error is set by
      the *angle* the window spans; a fixed ±100 m half-window spans four
      radians of a 50 m corner, and nodes under a 60 m radius were
      displaced a median 4.00 m — the deviation clamp, saturated. The
      window is now bounded by the road's own signed turning
      (`SMOOTH_MAX_TURN_RAD`), read over a chord long enough for
      digitising zigzag to cancel out of it rather than close it.
    - The band was cut at **whole mapped segments** while the deck began
      at the exact span arc, drawing bare ground at a quarter of all
      abutments, out to 27 m (`seam.abutment_bare`). `level_runs` now
      cuts in arc.
    - And the original item: the union is now buffered around the same
      smoothed line the structures sweep, sampled at the corridor's own
      solved stations, and at-grade paint rides it too. H2 holds —
      "every consumer must read the same one or decks, asphalt, and
      paint drift apart" — and the carry in `synth::road::bake` is
      unconditional again because there is no second curve to choose
      between.

    Montreux z16, HEAD → after: `seam.abutment_plan` median 0.98 m →
    **0.007 m** (the quantization floor) with 82 % → 27 % of abutments
    past 5 cm; `seam.abutment_step` 35 % → 22 % over; `seam.abutment_bare`
    16.4 % → 10.0 % over, worst 27.8 → 9.5 m; `paint.marking_offside`
    worst 165 m → 0.05 m.

    What it costs, and where to look if it bites: the junction `Area` is
    still built at the mapped connector point with mapped leg headings,
    so a mouth and the band that lands on it now disagree by the
    smoothing displacement (a median 0.45 m). That is within what the
    `Area` is for — it "only has to say roughly where the intersection
    is, for the marking trim, the height-field pin and the curb-return
    mask", since the union paves the real shape — but it is no longer
    the *by construction* agreement invariant 1 claims. The measurable
    cost is at **hairpins**, where smoothing pulls the two arms of a
    switchback towards each other: `order.grade_stack` 11.4 % → 12.2 %
    over and `slope.carriageway_face` worst 8.9 → 18.8 (both arms' bands
    merging into one region and the mesh ramping between them). Tightening
    `SMOOTH_MAX_DEV_M` trades that back against kinking, and now that the
    displacement is *shared* by every consumer it buys no consistency —
    only fidelity to the mapped line — so that is the dial to turn.

    *Increment 6 — the surface handoff (measured, not fixed).* Everything
    above was measured on the **strokes**, and at the zooms where the road
    surface actually exists there are none: from
    `priors::ROAD_SURFACE_MIN_ZOOM` a carriageway's own stroke is deleted
    because the union paves it (`pipeline::paves_via_union`), so every
    `seam.abutment_*` sample at z16 is a *railway* — a railway keeps its
    stroke, since its track is not a fill. Which inverts what the numbers
    mean: on a rail bridge that ribbon is drawn over both the ballast band
    and the deck, so the joint underneath it is hidden by the very thing
    being measured, while a road has nothing over the joint and shows every
    millimetre. Rail is the population with the bad numbers and the good
    picture; roads had the opposite and no numbers at all.
    `verify::checks::handoff` closes that: it anchors on the **band mesh's
    own silhouette** and marches out to the deck that continues it
    (`seam.band_deck_bare`, `seam.band_deck_step`). Montreux z16: 854
    joints, 586 on asphalt, median gap 0.10 m and median step 0.007 m —
    both the instrument's own floor, so the typical joint is flush and
    increment 5 holds for the surface as well as for the paint — with
    19.7 % of gaps past 0.15 m gathering into 58 places, 41 of them
    asphalt, and 16.5 % of steps past 5 cm. The worst are not
    seams but holes: at 6.9338,46.4081 the motorway band stops **10.7 m**
    short of its own viaduct, over a ravine 14 m below, and at
    6.9154,46.4400 a service road stops 7.0 m short. The cause is not the
    centerline — those joints are cut from one arc — so the next place to
    look is which spans the union declines to pave and which the sweep
    declines to draw.

    The stroke asymmetry this increment measured has since been closed:
    rail joined the road rule, so from `ROAD_SURFACE_MIN_ZOOM`
    `paves_via_union` deletes the rail stroke with the carriageway's and
    the ballast band meets its deck as nakedly as asphalt does.
    `slope.rail_grade` moved model-side with it (`verify::model::grade` —
    the solve is the only place the alignment still exists as a line), and
    `seam.abutment_*` reports its designed skip at the detail rung.

    *Increment 7 — no kerb line across a handover.* The rim exists to edge
    the paved surface against the ground it stops at (§6.1). Its skip rule
    exempted only *tile* cuts, so it also wrapped the abutment, drawing a
    `PAVE_RIM_M` line in the casing's darker tone across the carriageway a
    third of a metre before every bridge — on **98.5 %** of joints, measured
    by `seam.handover_kerb`. The deck carries no matching rim, so the joint
    read as a border rather than as a road. `synth::junction` now records
    the cut where an at-grade run ends at a span boundary (`Handover` — the
    only stage that still knows, since the union is a boolean over buffered
    polylines and dissolves which input each stretch of boundary came from),
    the pavement bake files them per chunk, and `build_rim` sends those
    quads to the **surface** instead of the casing. They keep their geometry
    — the interior is inset and something has to cover the strip — and lose
    the tone and the across-coordinate with it, so the asphalt runs into the
    deck unbroken.

    Same bbox, A/B by `ARPT_KERB_AT_HANDOVER=1` (which withholds the cuts,
    the way `--no-hole` withholds the hole): `seam.handover_kerb`
    98.5 % → **15.0 %**, and 37 of the 54 edges left are on joints
    `seam.band_deck_bare` also calls broken — where the band's edge is not
    at the span boundary at all, so no cut lies on it and the kerb line is
    correct. The real residual is nearer 5 % — 17 edges, not yet attributed;
    a curb-return fillet reshaping the cut and a corridor the solve gave no
    profile are the two candidates. `seam.band_deck_bare` reads 21.7 % → 16.4 %
    over, mostly because the anchor moved: the interior mesh now reaches the
    true silhouette at a handover, so the march starts a rim-width further
    out. The one cost is `slope.carriageway_face`, worst 18.4 → 20.7: those
    rim quads were excluded from it as kerb, and now that they are surface
    they are counted — the geometry did not move, only the population, and
    the new worst is a pre-existing 7 m height jump in the band at
    6.9121,46.4333 that the casing was already drawing.
    *Increment 8 — the deck is the same asphalt.* Invariant 1 says one
    cross-section; the colour did not follow it. A structure's top is
    painted per-vertex from the style entry its **road class** names
    (`decode.c`, `fs_deck`), while the band beside it is painted from
    `road_surface` — so a residential deck came out 166,168,172 against the
    band's 151,154,159 and the tone stepped at every abutment. The deck
    stopped reading as a continuation of the road and started reading as a
    different object laid over it, which no amount of geometric agreement
    fixes. Structures now carry `band_class`, naming the band their running
    top *is* (`road_surface` / `rail_surface`), and the client paints from
    that, falling back to the road class where there is none. The modality
    is decided server-side because only the server has it: the archive
    carries a class but not its subtype, and a railway deck belongs to the
    ballast band. Montreux z16, small-bbox retile: 46 of 47 drawn decks name
    their band — the 47th is a class that paves nothing, which correctly
    names none — and no geometry metric moves (`seam.band_deck_bare` 359
    samples, 16.4 % over, before and after).

    *Increment 9 — one shape cuts the other.* Two ends cut from one arc
    still did not meet, because each side turned that arc into geometry
    its own way. The deck lays down an **explicit cross-section**
    (`Profile::deck_nodes`: a point, a left vector, a half-width); the
    band gets whatever `poly::buffer_line` butts onto the **last chord**
    of its polyline, a station's worth of curve back. Those two lines
    cross at the centreline and diverge to either kerb — bare ground at
    one, an overlap at the other, `half_w · θ` wide for the turn θ that
    chord spans, which is 0.4 m at the profile's ~4 m stations on a 25 m
    ramp radius. It reads as a thin wedge just before the bridge, which
    is exactly how it was reported.

    The confirmation was in the samples the check was *dropping*: of 43
    gapping joints, 40 had an overlapping edge within 12 m of the same
    cap, and 441 edges archive-wide were already buried under their deck
    (`ARPT_DEBUG_OVERLAP`). A signed defect hides half of itself in
    whatever exclusion the metric applies.

    Two patches were built and both are wrong in the same way. Making the
    band's last chord short enough to *be* the tangent (a 0.25 m cap
    chord) fixes only the angle, and took the small gaps 31 → 22 —
    because the cap angle is one of several things two constructions can
    disagree about. Overshooting the boundary by 0.5 m took them 31 → 7,
    but only by burying every residual disagreement under the deck: the
    measurable joint count fell 359 → 248, which is the instrument going
    blind rather than the surface getting better.

    What landed instead: `synth::junction` builds the handover cut from
    `deck_nodes` — so the line *is* the deck's end face rather than a
    second derivation of it — buffers the band `STRUCTURE_OVERRUN_M`
    (1.5 m) **past** the boundary, and `synth::pavement::bake_chunk`
    subtracts everything beyond that cut, per run, before the union.
    Generate long, trim to the thing it must meet. The shared edge is one
    set of coordinates because one shape cut the other, which is a
    different kind of agreement from two constructions arriving at the
    same place. The trim is per *run* because that is the last moment the
    model still knows which band a cut belongs to: after the boolean the
    region has dissolved it, and a cut applied there would as happily
    take a bite out of the road passing underneath.

    Montreux z16, A/B by `ARPT_NO_ABUTMENT_CUT=1`: small gaps
    (0.15-1 m) **31 → 7**, violations 59 → 32, and the site reported from
    the viewer 5 → 0 — the overlap's result without the overlap, and with
    the joint count held at 351 so the joints stay measurable. The ≥1 m
    population is untouched (28 → 25): that is the band stopping metres
    short, a different defect. `contact.kerb_lip` returns to 13.79 %,
    where the overshoot had cost 0.03 pp. Two metrics read slightly worse
    and both are the measurement improving rather than the surface
    degrading: `seam.band_deck_step` 16.7 → 17.7 %, because the step is
    now compared at the true joint instead of a rim-width back, and
    `order.deck_above_carriageway` 0.92 → 1.03 %, the band's edge now
    landing exactly under the deck's.

    What it retires is as much the point as what it fixes: with the band
    cut by the structure, there is no cap angle to match, no chord length
    to tune, and no threshold anywhere in it.

    *Increment 10 — a street is a room between facades (done
    2026-08-22).* Nothing owned a street's cross-section. The carriageway
    took the band its class prior asked for, the bench took a wider one,
    and both were drawn through whatever wall stood there: **28,719 m²
    of drawn at-grade asphalt inside 1,662 of Montreux's 8,615
    footprints** (`order.building_overlap`, 0.985 % of samples past
    `FACADE_CLEAR_M` at z16, 1.197 % at the coarse rungs).

    The obvious fix was measured and rejected before it was built.
    Widening the bench to reach the sidewalks beside a street would drive
    a road earthwork under a facade on **19.7 %** of residential street
    length, and a building anchors at the highest drawn ground under its
    footprint — a bench under a wall sets that anchor from a road
    earthwork and the building rides up on it. The room to widen into is
    not there.

    So the cross-section is **allocated out of the room the buildings
    leave**, not asserted from the class prior. `assemble::facades` reads
    every footprint edge into a grid index and answers one question:
    standing at this station on this centerline, how far is the nearest
    wall to my left, and to my right? It is a *cross-section*, not a
    proximity — only the stretch of wall within a window along the
    centerline counts, and it counts by its lateral offset, so a building
    at the head of a cul-de-sac does not narrow the street leading to it.
    The window is at least the gap to the neighbouring stations, which is
    what makes consecutive stations see the same wall: a facade is caught
    by every station whose window it falls in, so the two bracketing its
    ends are both narrowed and the width interpolated between them never
    crosses it.

    `synth::carriageway` spends that room at bake time — the class prior,
    capped by `room − FACADE_CLEAR_M`, floored at
    `MIN_CARRIAGEWAY_HALF_M` — and bakes the result onto each source as a
    per-end, **per-side** `Section`. Per side because a wall stands on
    one side of a street far more often than on both, and a symmetric cap
    would pay for one close facade by narrowing the open side too. A run
    whose section is uniform still goes through the constant-width stroke
    it always did, so the joins on the network nothing crowds keep their
    exact miters; a run that varies is built by `poly::buffer_section` as
    a union of convex pieces — one trapezoid per segment, one patch per
    join — which has no self-crossing case to arbitrate, at the cost of
    beveling the outside of a join instead of mitering it (under a
    centimetre at the profile's ~4 m stations).

    The floor is not a safety margin, it is a **classification**. The
    population has two families and only one is a width problem: a street
    a wall crowds, which the room narrows, and a way whose *centerline*
    runs inside a footprint — a parking structure's service aisles, the
    Casino Barrière's 7,533 m² footprint with an `unknown`-class way
    through it — which no cap can move and which subtracting the
    footprint would cut into disconnected pieces. The floor keeps that
    second family a road, and the check keeps reporting it.

    Montreux, control tiled from the same commit: `order.building_overlap`
    **0.985 → 0.293 %** at z16 and **1.197 → 0.371 %** coarse — 71 % and
    69 % of the violations gone — with the worst unmoved at 28 m, which
    is the second family staying exactly where it was. At Rue du Marché,
    6.074 → 1.502 % and the worst 3.89 → 2.18 m. `contact.kerb_lip`
    (−5.7 % violations), `order.deck_above_carriageway` (−3.5 %) and
    `slope.carriageway_face` (−9.4 % at z16) improved with it; the paved
    area fell 0.69 %, and `paint.edge_line_inset` did not move, edge
    lines being a motorway-network marking and a motorway having the room.

    *Still open:* the **bench** is not yet allocated from the room — only
    the asphalt is. Between the narrowed kerb and the unchanged earthwork
    edge there is now drawn ground at bench level where asphalt used to
    be, which is why `slope.terrain_tearing` picked up 5.3 % more
    violations at z16 on a population that grew 0.24 %. Per-side bench
    half-widths, the walk band that belongs between them, and the
    facade-clipped batter are the phases after this one.

    *Increment 11 — the bench's cross-section, built and left switched
    off (2026-08-22).* `EarthworkEdge::half_width_m` is per side, the room
    is resolved once per profile node at derive time and baked onto each
    edge, and `Room::allot` is the one function the asphalt and the ground
    both spend the room through (invariant 1). `ARPT_FACADE_BENCH=1` turns
    the clip on; `ARPT_FACADE_BATTER=1` adds the face's.

    **It is off because the measurement says it is not yet a net
    improvement, and the reason is a phase that has not landed.** Tiled and
    scored against the same control:

    | | bench clip | + batter clip |
    |---|---|---|
    | `authority.facade_ground` | 1.934 → 1.096 % | 1.934 → 0.656 % |
    | wall samples past a metre | −2,225 | −3,408 |
    | `contact.kerb_lip` | 6.80 → 7.70 % | 6.80 → 8.63 % |
    | kerbs gaining a drop | **+3,850** | **+7,828** |

    Both halves of the trade are 1 m-scale defects on comparable
    populations (261 k wall samples against 429 k kerbs), and both times
    the cost is larger than the gain. `slope.terrain_tearing` and
    `contact.kerb_unwalled` improve either way; `contact.sidewalk_grade`
    and `slope.terrain_face` follow the kerb.

    The mechanism is mechanical once seen: `contact.kerb_lip` probes one
    metre outside the kerb, so narrowing the bench moves that probe off
    the verge and onto the batter face. *Any* narrowing costs it, whatever
    the room says — until something occupies the strip between the kerb
    and the facade. That something is the walk band (P5, "adjacent
    infrastructure"), riding the host's cross-section at `KERB_RISE_M`.
    So the phase order in the plan was right and the dependency runs the
    other way from how it looked: the bench cannot be allocated out of the
    room before the sidewalk is there to receive what it gives up.

    One thing the build settled on its own: the ground stops **at** the
    wall, not a clearance short of it. `FACADE_CLEAR_M` keeps a drawn
    surface off a footprint; a wall stands *on* ground, and a street
    between buildings is flat from facade to facade. Clipping the bench
    half a metre short was built first and read `contact.kerb_lip` 9.08 %,
    worse than clipping at the wall.

    *Still open:* the deck top has the analytic edge AA (`sweep_prism`
    already writes ±1 `edge_across` on its two edge strips) but not the
    band's **kerb line**, so the rim runs along the approach and stops at
    the abutment. Closing it means two extra longitudinal vertex rows at
    ±(half-width − `PAVE_RIM_M`) with a piecewise `edge_across`, a second
    per-vertex colour on the structure pipeline for the casing tone (the
    style already parses `casing_color` per class), and `fs_deck` picking
    between them. It is a GPU vertex-layout change and a purely visual one,
    so it wants a render in front of it.

- **P4 — Symbols.** The MSDF atlas: gore chevrons from diverge geometry;
  crossing glyphs where `cycle_crossing` meets a carriageway. Turn arrows
  are deferred — the schema carries no turn data (P0); they wait on an
  OSM-side `turn:lanes` enrichment or the lane model's return to Overture.
- **P5 — Adjacent infrastructure.** Sidewalks and cycleways as surface
  ribbons of their own classes; kerb lines; medians and islands as
  negative space the paint respects.
