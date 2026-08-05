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
   share one prior; this widens that contract.)
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
  - *Remaining:* symbols (arrows, chevrons — the MSDF atlas, P4);
    distance fade — sub-texel lines shimmer at grazing angles; stop
    lines and zebra crosswalks from `crosswalk` segments; dividers on
    wide two-way roads (2×2 boulevards).
- **P4 — Symbols.** The MSDF atlas: gore chevrons from diverge geometry;
  crossing glyphs where `cycle_crossing` meets a carriageway. Turn arrows
  are deferred — the schema carries no turn data (P0); they wait on an
  OSM-side `turn:lanes` enrichment or the lane model's return to Overture.
- **P5 — Adjacent infrastructure.** Sidewalks and cycleways as surface
  ribbons of their own classes; kerb lines; medians and islands as
  negative space the paint respects.
