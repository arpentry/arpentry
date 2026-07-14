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
   far side. The junction plates (`synth/junction.rs`) already concede this:
   a filled area is meshed precisely because overlapping strokes cannot
   express it. This document generalizes that concession to the whole
   network at detail zooms.

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
resets at a tile seam produces a visible stutter. As with piers placed at
global multiples of `PIER_SPACING_M`, dash phase must be a function of
*global* corridor arclength, never of the tile window (GENERATION.md
invariant 5).

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
2. **A closed, simple silhouette.** The union of corridor surfaces and
   junction plates has no gaps, no slivers, no overlapping fills: legs and
   plates share their boundary exactly.
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

- **The surface is a mesh.** Baked in synth like decks and plates already
  are: the corridor centerline buffered by the width function, unioned with
  junction plates, corners filleted, tessellated, draped on the engineered
  ground, emitted as `MeshGeometry`. The silhouette — fillets, tapers,
  gores — is real geometry, crisp by construction.
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
4. **Synth** gains the surface baker (corridor polygons unioned with —
   and eventually absorbing — the junction plates) and the marking baker
   (band-boundary strips, stop lines, crosswalk boxes, symbol quads). Both
   read solved heights and the engineered ground; markings on structure
   decks read the same road-surface function, so a bridge carries its lane
   lines across (GENERATION.md invariant 2).
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
  - *Increment 2 — junction trim and fillets (done 2026-07-14).* Plate
    corners curve as curb returns (quadratic Bézier at the carriageway-edge
    intersection, straight-chord fallbacks for flat sides and reflex gaps);
    plate legs span the band edge — the P1 width (mapped where plausible)
    plus the shoulder — and surface bands are interval-trimmed at each
    plate's disk (`BakedJunction::trim_radius_m`, endpoint and pass-through
    cases alike), tucked `BAND_TUCK_M` under the mouth so no ground sliver
    can open.
  - *Remaining:* absorb the plates and bands into one unioned surface per
    junction (today they abut mouth-to-mouth); `road_flags` ingestion.
- **P3 — Longitudinal paint.** The marking baker and the client decal
  pipeline: centre/lane/edge lines as procedural-dash strips, stop lines,
  zebra crosswalks (procedural stripes registered to R7's carriageway
  span).
- **P4 — Symbols.** The MSDF atlas: gore chevrons from diverge geometry;
  crossing glyphs where `cycle_crossing` meets a carriageway. Turn arrows
  are deferred — the schema carries no turn data (P0); they wait on an
  OSM-side `turn:lanes` enrichment or the lane model's return to Overture.
- **P5 — Adjacent infrastructure.** Sidewalks and cycleways as surface
  ribbons of their own classes; kerb lines; medians and islands as
  negative space the paint respects.
