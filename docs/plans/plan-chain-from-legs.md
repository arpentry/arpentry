# The plan chain from explicit geometry (review step 5)

Written 2026-09-27 at the end of the session that landed review steps 0–3 and
their follow-ups (`9f14edb`…`23d10d2` on `spans-are-derived`). This document is
the hand-over for the session that starts step 5. Read it, then the "World
crate" section of `CLAUDE.md`, before touching code.

## Why this step

The review of `world/` found one root cause behind most of the model's small
defects: **every fact is computed more than once, in different
representations, and reconciled afterwards.** Steps 0–3 removed that in the
3D half:

- **One mesh.** The paving and the ground share their vertices by index, and
  seams are read by index rather than by position.
- **One rule per triangle.** Every step in the lift is either welded or drawn
  as a wall.
- **One continuous earthwork residual.**

The 2D half is untouched, and it is now where the leftover defects come
from. A centreline becomes a paved polygon through `ribbon → surface → kerb →
fillet → room → sheet`:

- **About 40 whole-surface booleans**, plus hundreds of local ones.
- **The same piece is buffered about four times**, with three different end-cap
  rules: `ribbon::caps`, `surface::joints`, and the round `buffer_line` in
  `sheet`/`facade`.
- **Provenance is lost.** A union forgets which road made which part, and later
  steps sample points to guess it back (`fillet::kerbs_at`,
  `sheet::sample`/`nearest`, the arrangement's probes, room's scrap
  attribution).

What that produces, measured at HEAD (`23d10d2`):

| Where | What | Loop box | Junction box |
|---|---|---|---|
| room | `scraps` — regions too narrow to be pavement | 895 (268.9 m²) | 101 (40.2 m²) |
| room | `loose` — detached lobes, round caps cut from their ribbon | 145 (110.1 m²) | 17 (14.1 m²) |
| sheet | `orphan` — paving no piece claims | 7 | — |
| arrangement | `unprobed` — faces too thin to hold a point | 1 813 | 198 |
| mesh | `crack` — one-sided edges off the rect border | 21 m | 1.6 m |
| mesh | `slivers` — triangles under 1e-6 m² | 70 089 | — |

The remaining cracks are the telling ones. They are 45 faces of zero-area
slices and edges whose two sides genuinely disagree by decimetres. That is
geometry the plan chain built twice and differently, and no repair
downstream can make it agree.

Then there is the clean-up layer whose only job is to remove the output of
the layers before it:
- `fill_holes_under` ×3
- `OPEN_M`/`OVERLAP_M`
- `wide_enough`
- `loose` and `WALLED_SHARE`
- `adopt`
- `poly::snap_twins`
- `mesh::cleaned_together`, `close_t_junctions` and `weld_open`

Step 5 is where the chain stops producing what these clean up.

## The target

Build the plan from **explicit geometry that remembers where it came from**:

- **Each road piece gets offset lines.** Its kerb lines are the centreline
  offset by half its width on each side.
- **Each junction is a polygon built from the legs meeting there.** Adjacent
  legs' facing kerbs are joined by the return arc of the narrower leg's
  fillet radius. Each leg's mouth is cut square where its arcs end.
- **Legs are trimmed at their mouths.** No round cap is drawn at any connector,
  so no cap survives as a lobe.
- **Sidewalks are bands between a kerb line and an offset of it**, replacing
  rungs, fans and `fill_holes` once the explicit version measures better.
- **One labelled planar subdivision is built from all of it**, with every face
  tagged by the set of sources that made it, instead of `poly::slice` over
  unioned regions followed by a point probe.

Buildings stay a boolean: the facade clips what it clips. Grade separations
stay per sheet: two sheets are never unioned.

## First slice: junction polygons, measured and not wired

The repository's discipline, and the one that held through steps 0–3: **build
beside, measure against today's output, and wire only when the measurement
says so.**

1. **A module that builds one polygon per junction** (name it; `junction.rs`
   is taken by an older set of `#[ignore]`d checks). Its inputs:
   - the ground carriageway pieces meeting at each connector of ≥ 2 pieces;
     `surface::joints` already counts them per family;
   - each piece's `width_m`;
   - `width::fillet_m`, the fillet radius prior, which is per class and
     takes the narrower leg.

   The construction:
   - Sort the legs by bearing.
   - For each consecutive pair, intersect the facing kerb offsets. Where the
     turn is at least `fillet::BEND_MIN_DEG`, replace the corner with a
     tangent arc; below that, join them straight. Clamp the tangent length
     at `fillet::RETURN_MAX_RADII` radii.
   - The mouths go where the arcs end.
2. **A comparison per junction against today's result.** In a disc around
   each connector, measure:
   - the symmetric difference between (junction polygon ∪ trimmed legs) and
     `fillet(world).surface.carriageway`, as area and as a max distance;
   - the count of today's regions there that `room` or `mesh` later reports
     as `scraps`, `loose`, `unprobed` or sliver.

   Report it on a summary line, the way every other step does.
3. **The hard cases, each with a specimen before a rule:**
   - A leg shorter than its tangent length, so the arcs overlap.
   - Connectors closer than a junction's size. This covers dual-carriageway
     crossings and a roundabout's ring meeting its legs at several close
     connectors; they probably have to be one junction.
   - Legs meeting at near-collinear angles, where the offsets' intersection
     runs far away.
   - Interior connectors: `crossing` and `profile::contacts` show that
     Overture does not cut a way at every connector, and the 48 rail × road
     level crossings on the loop box are all interior.
   - Spans at a junction. The sheet grouping must still see one surface.

   `net:` has most of these: `cross`, `tee`, `tee?d=8&hook=5`,
   `roundabout`, `dual?gap=4`, `hairpin?angle=20`, `corner`, `level`.
4. **Wire it only if the comparison says the explicit polygon is at least as
   good everywhere and strictly better where the counters point.** Then
   `fillet`'s closing machinery (`OPEN_M`, `OVERLAP_M`, `kerbs_at`) should
   have nothing left to do, which is the proof.

## First slice: status (2026-09-27)

**Wired 2026-09-28** (see the end of this section); what follows is the
history of getting there. `world/src/legs.rs` is the `legs` step, after
`fillet`. Until it was wired nothing read its layer. The plan view draws
it (`legs` group: junctions outlined blue, `extra` green, `missing` red).

The construction, and the three things the first version got wrong:

- **A node** is a connector two piece vertices share. A piece is cut into
  edges at every node it passes, so an end on another's interior vertex is
  a junction too.
- **Kerbs are offset polylines, not rays.** Built on the first segment's
  ray, every curved leg disagreed: the roundabout read 2.44 m at all four
  junctions. The return's centre is where the two kerbs offset by `h + r`
  cross, and its tangent points stand beside it on each kerb, which is exact
  on a curve. Roundabout: **0.04 m**.
- **A kerb point at an arc is square off the centreline there**
  (`Kerb::at`), because that is where the edge's butt end puts its corner.
  Interpolated between mitred vertices, every curved mouth left a hairline.
- **The clamp reads the tangent point's real reach** (≤ `RETURN_MAX_RADII`
  radii along either kerb from the corner) and shrinks the radius by
  bisection. Read off the turn at the corner, a leg that bends a few metres
  out made a 104° corner of a 19° fork, and the return sat 6 m up the V.

**Measured beside `fillet`:**

| | junction box | loop box |
|---|---|---|
| junctions / legs | 60 / 184 | 829 / 2 494 |
| `extra` / `missing` m² | 17.5 / 103.1 | 340 / 3 434 |
| `apart` p50/p90/max | 0.16 / 2.36 / 3.41 | 0.18 / 2.70 / 7.81 |
| `far` / `short` / `tangled` | 5 / 7 / 2 | 96 / 96 / 45 |

**The temporary A/B** (not committed): `room` fed the explicit carriageway,
with the walk cut by it. Loop box:

- **Better:** `scraps` 895 → 669, `loose` 110.1 → 87.1 m², `unprobed`
  1 813 → 1 561, `wall_gap` 28 → 17, `wall_m2` 10 886 → 10 112,
  `kerb_m2` 32 715 → 30 702, `split_m2` 19 550 → 17 569. The junction box
  agrees (`scraps` 101 → 83, `unprobed` 198 → 147); the roundabout's `step`
  went 3 → 1.
- **Worse:** `kerb_gap` 29 → 44, mesh `slivers` 70 089 → 70 546, `crack`
  21 → 22 m, `unmet` 0 → 2, `orphan` 7 → 8, galleries 20 → 25, decks
  49 → 46.

So it is not yet "at least as good everywhere". The first pass named three
blockers, and all three are now built (with a specimen each in
`legs::tests`):

1. **Junctions on a deck.** The node graph is built over the ground pieces
   *and the bridge spans* (never a bore, as `spans_grouped`), so a junction
   standing on a deck is a junction. A span edge shapes its junctions and
   is not paved here; a junction with a deck among its legs paves only what
   the span ribbons do not already pave — its returns, as `fillet` does.
2. **The pavement is laid back.** The step returns a whole `Surface`: the
   walk grows by `kerb::WALK_MIN_M` wherever the new asphalt covered it,
   and is cut to the new senior surface — `fillet`'s rule, read off the
   result. Its own `kerb_gap` is on the line.
3. **An end that lands on a road** with no shared vertex (`landed`): the
   host gets a vertex at the end's foot (or its nearest vertex within
   `LAND_SNAP_M`) and the end is carried there. 10 on the loop box.

And `fillet`'s span-group closing now reads its own group's asphalt
(`b8f3c23`): the overpass's four returns under the deck are gone.

**The A/B after all three**, loop box, against `b8f3c23`:

- **Better:** `scraps` 895 → 668, `loose` 110.1 → 96.0 m², `unprobed`
  1 813 → 1 752, **mesh `crack` 21 → 8.4 m**, `wall_m2` 10 886 → 10 192,
  `kerb_m2` 32 715 → 31 014, `split_m2` 19 550 → 17 873.
- **Worse:** `kerb_gap` 29 → 61, `wall_gap` 28 → 51, `orphan` 7 → 18,
  `slivers` 70 096 → 70 701, decks 49 → 46, galleries 20 → 25, `unmet`
  0 → 2.

**Then the kerb walk (close junctions).** A leg's kerb no longer stops at
its far node: it goes on as the boundary of the face on that side — at each
node the edge first clockwise from the one it arrived on (left kerb) or
first counter-clockwise (right kerb), up to `WALK_M` or `WALK_HOPS` — so the
return between a leg of one junction and a leg of the next is found from
either. A mouth past the leg's own end is drawn across the leg at its far
node, and an edge its mouths overrun is paved whole.
`a_return_reaches_past_the_next_node` is the specimen, and fails with the
walk off. **Merging the two nodes into one junction was tried first and
reverted**: the link between them stops being a leg, and the return between
it and an outer leg is lost instead.

Three defects the walk exposed, fixed where they live:

- **Vertex twins.** `689f09` on the loop box has two vertices 3 mm apart
  that round to two connectors; the edge between them has a direction that
  is noise, and the walk followed it back up the road. Consecutive vertices
  under the connector snap collapse to the one another piece shares
  (`close_twins`), and a walk never takes an edge doubling back on the one
  it arrived by.
- **Deck junctions** pave off the *square* deck footprint
  (`surface::spans_masked`, the bench's mask), so the round of the node the
  ground ribbons paved survives, and the remainder is opened by `LAP_M`:
  a difference along the deck's kerb left hairline fragments that were most
  of `sheet`'s new orphans.
- **A mitre corner far from its node** (two kerbs of different widths at a
  grazing angle, often a stub the clip left) falls back to the round join.

**The A/B now**, loop box, against `b8f3c23`:

| | fillet | legs |
|---|---|---|
| `scraps` | 895 | 661 |
| `loose` | 145 (110.1 m²) | 157 (104.8 m²) |
| `unprobed` | 1 813 | 1 644 |
| `kerb_gap` | 29 | 43 |
| `wall_gap` | 28 | 64 |
| `orphan` | 7 | 13 |
| mesh `crack` | 21 m | 8.4 m |
| mesh `slivers` | 70 096 | 71 090 |
| decks / galleries | 49 / 20 | 47 / 25 |
| `unmet` | 0 | 2 |
| `wall_m2` / `kerb_m2` / `split_m2` | 10 886 / 32 715 / 19 550 | 10 249 / 31 281 / 18 122 |
| carriageway regions/holes | 60 / 160 | 73 / 188 |

`missing` 3 473 → 3 021 m², `extra` 390 → 582 m². Still not wired. What is
left:

1. **Narrow forks and small islands are paved — decided (2026-09-27).**
   The returns pave a fork up to their reach (3 radii along either kerb)
   and an island's corners; `room` paves what is left of an island as a
   traffic island if it can hold the narrowest pavement, and `legs` fills
   every hole of the carriageway that cannot (`room::wide_enough`, shared;
   `98cae84`). A closing by half the pavement width over the whole surface
   was tried first and reverted: it re-rounded every kerb (mesh slivers
   71 k → 110 k, crack 8 → 93 m).

**Then, 2026-09-28, the open items:**

- **The pavement follows the kerb inward** (`3ae8f1f`): where the explicit
  kerb stands inside the old one, what the old asphalt covered within the
  narrowest pavement of the walk is the walk's, lapped by a centimetre so
  it joins. `kerb_gap` 39 → 30.
- **A junction is one region** (`6265611`): where its ring crosses itself
  the union handed back lobes of a few square centimetres, or three
  coincident points — asphalt no piece claims. `orphan` 13 → 4.
- **`fillet`'s closing across grade separations is not fixed in `fillet`.**
  Run per group (`partition::groups`), it stopped crossing grade
  separations but also lost the returns at ends that land on a road with
  no shared vertex — different groups — and `kerb_gap` rose 29 → 40.
  Reverted: telling the two apart is what `legs` does, and wiring it is the
  fix.
- **Gores between parallel minor roads were tried and reverted.** Paved
  past the return while narrower than the narrower road, stopped at walks
  and buildings: `loose` 145 → 167 (97 → 145 m²), `orphan` 13 → 15, holes
  156 → 173, and `wall_gap` did not move — the garage court's strips lie
  between lanes of *different* junctions, which a gore at one node does
  not reach. The fork rule stays: paved to the return's reach.
- **Decks 49 → 45 and galleries 20 → 25 are face counts, not structures.**
  `deck_m2` falls 22 m², which is the fillet's ground asphalt under decks —
  its defect, which the explicit construction does not have; `spanned`
  and gallery faces are the same area cut into more pieces.
- **`unmet` 2** is one 1.2 m open mesh edge where a building corner cuts the
  explicit carriageway (loop box 837, 1371): a residual crack, while the
  box's total `crack` falls 21 → 8.4 m.

**The A/B at `6265611`**, loop box, `room` fed the explicit surface:

| | fillet | legs |
|---|---|---|
| `scraps` | 895 (268.9 m²) | 661 (265.8 m²) |
| `loose` | 145 (110.1 m²) | 142 (92.0 m²) |
| `unprobed` | 1 813 | 1 632 |
| `kerb_gap` | 29 | 27 |
| `wall_gap` | 28 | 35 |
| `orphan` | 7 | 4 |
| carriageway regions/holes | 60 / 160 | 64 / 155 |
| mesh `crack` | 21 m | 8.4 m |
| mesh `slivers` | 70 096 | 71 274 |
| `unmet` | 0 | 2 |
| `wall_m2` / `kerb_m2` / `split_m2` | 10 886 / 32 715 / 19 550 | 10 225 / 31 393 / 18 240 |

**The three wall-gap sites, fixed (2026-09-28).** Each was read with a
coverage grid of the site, and two of the three were `room`'s, not the
construction's:

- **A footbridge's stub** (`f2c46d7`, `room`): a footway's ground piece
  between the kerb and the footbridge it climbs is a few square metres with
  no wall, and was dropped as a scrap. `room` now gets the walk spans and
  keeps a piece that reaches one. Under the fillet too: `wall_gap` 28 → 22,
  `kerb_gap` 29 → 22.
- **A small court** (`1ba1da2`, `room`): the strip between two garage lanes
  and the garage they end at is enclosed on every side, and was dropped as
  a scrap. A region with asphalt, ballast, a span or a wall just outside
  every edge is now kept.
- **Converging carriageways** (`legs`): a gap between two carriageways that
  no pavement fits in is asphalt, paved as quads between rays from the
  carriageway's own edge — only the gap moves.

**The A/B now**, loop box, against the fillet at `1ba1da2`:

| | fillet | legs |
|---|---|---|
| `scraps` | 895 | 670 |
| `loose` | 138 (87.8 m²) | 130 (83.7 m²) |
| `unprobed` | 1 817 | 1 632 |
| `kerb_gap` | 22 | 21 |
| `wall_gap` | 22 | 12 (none new) |
| `orphan` | 7 | 4 |
| carriageway regions/holes | 60 / 160 | 58 / 161 |
| mesh `crack` | 21 m | 8.4 m |
| mesh `slivers` | 70 116 | 71 212 |
| `unmet` | 0 | 2 |

**The last two, read (2026-09-28), and the slice wired.**

- **`unmet` 2 was `poly::conform`, not a building** (`8ceb3bf`). It tested
  each vertex against the segment as it was before the pass, but a vertex
  put on a segment bends it by up to a grid step: at (979, 163) the next
  vertex lay 8.8e-5 m off the new piece and 1.3e-4 m off the old edge, and
  was skipped. It now repeats until nothing is inserted. Legs `unmet`
  2 → 0, `crack` 8.4 → 3.2 m; the fillet's 21 m did not move.
- **Mesh slivers are systematic, and the mesher's.** Legs − fillet is
  +1.6 / +0.8 / +0.8 / +1.6 % at spacings 3.45 / 3.6 / 3.9 / 4.3 m — never
  the other sign, so not lattice-phase noise, and not a site: +9 076 /
  −7 980 over ~3 000 20 m cells. It goes with +48 k carriageway triangles
  over *less* area and boundary: fewer, larger faces. **Vary `--spacing`
  above the vertex cap** (≈3.45 m on the loop box): below it every spacing
  is the same run. At 4.3 m the legs' `crack` is 8.9 m against the fillet's
  8.0, which the one-lattice A/B had hidden.
- **Where slivers come from (read on the wired loop box, 57 888 of them).**
  52 % are *interior* (centroid more than 1 µm off every ring) with **no
  ring vertex among their corners**, and 62 % have an edge under 10 µm:
  the clipper's diagonals crossing the lattice near a lattice vertex, where
  column, row and diagonal cut within microns. Only 13 % sit on a boundary
  with no ring vertex. (A 1 mm "on the boundary" test read 88 %, which is
  every thin sliver beside a kerb, and is wrong.) **Two boundary fixes were
  built and measured, and reverted:** snapping ring vertices within 1e-4 m
  of a lattice line onto it (−4.5 %; 1e-3 barely more), and putting a lattice
  vertex into any segment passing within 1e-4 m of one (−5 slivers). Neither
  moved `crack`, `unmet`, `seam` or `lost_m2`. Welding wider is the refuted
  route in `mesh::WELD_M`'s doc. **The root fix is to cut first and
  triangulate after**: split each face by the lattice into pieces inside one
  lattice triangle, then triangulate each piece, so no clipper diagonal ever
  crosses a lattice line. That is a mesher redesign, not a threshold.

**Wired**: `room` reads `legs.surface`. Loop box against `8ceb3bf`:
`scraps` 895 → 670, `loose` 138 → 130, `unprobed` 1 817 → 1 632,
`wall_gap` 22 → 12, `kerb_gap` 22 → 21, `orphan` 7 → 4, mesh `crack` 21 →
3.2 m, `unmet` 0, `wall_m2` 10 884 → 10 223, `kerb_m2` 32 724 → 31 669,
`split_m2` 19 553 → 18 559; slivers 70 127 → 71 171 (the mesher's, below). The flank roundabout's
last 2 steps go, and `ground::tests::every_drawn_edge_is_welded_or_walled`
is live. The junction box's `kerb_gap` is 4 → 5.

**Bench `fill` rose 15.59 → 19.76 m, and it is not a new defect.** It is a
facade band `room` draws from the ground carriageway's kerb at the deck
approach at (−639, 413) to a house 5 m off; the approach itself stands
17.6 m over the natural ground there, and the fillet's build had the same
case 4 m away at 13.8 m. Where a band lands beside a tall approach
embankment is the maximum, and the embankment is the span's extent, not the
junction's.

**Tried and reverted: a pocket that borders a span is not a pocket.**
`room::pockets` takes `carriageway ∪ spanned` as what encloses a hole, so
a deck's side bounds islands. Refusing every hole with a span-bordered edge
removed 11 islands (1 897 m²) but did not touch the site above (a band, not
a pocket), and refused a 381 m² hole at (456, −889) with 1 of 35 edges on a
span — an abutment's round cap grazing a real pocket. A rule here needs the
deck's height or its square footprint (`surface::spans_masked`), not any
contact.

**`fillet` is deleted (2026-09-28).** `legs::run` no longer takes the
fillet's surface, and its comparison counters (`extra`, `missing`, `apart`,
`near_m2`, `elsewhere_*`, `disagree`) went with it. Porting the fillet's
tests found one thing the wiring had dropped: **a way that turns sharply at
one of its own vertices got no return**, because `legs` rounds corners only
at junctions and `fillet` found them in the union's boundary.
`legs::bend_returns` gives the inside of every vertex turning at least
`BEND_MIN_DEG` the return a junction corner of that turn gets, at the way's
own radius, shrunk to fit the kerb the vertex has to itself: `bends` 311 on
the loop box, carriageway +98 m², `kerb_m2` 31 669 → 31 561, `split_m2`
18 559 → 18 455. The cost is `wall_gap` 12 → 18 and `scraps` 670 → 683, and
the six gaps are one site (−365, 1018): a return 1.3 m past a junction eats
a footway's round cap, and the laid-back pavement leaves a C-shaped scrap
across bare ground from the kerb. The laid-back rule is the one to fix there,
not the return. The ported specimens are `legs::tests`' bend, roundabout,
hook and narrower-way tests.

## After the first slice, in order

1. **Sidewalks as bands from their attachments** (`kerb::Attached` already
   knows which road and which side). Retire rungs, fans and `fill_holes_under`
   when `scraps` and `loose` measure 0 without them.
2. **A labelled slice upstream.** Slice the rect once by every explicit
   boundary and tag each face with its sources: a face covered by one leg is
   that leg's; a face covered by two is a junction; covered by none but paved
   is a pocket. `arrangement::adopt`, `poly::snap_twins` and the probes
   should then have nothing to do.
3. **Retire the clean-up layer, counter by counter.** A heuristic goes when
   the thing it cleans measures 0 without it — never before.

## Also open, from the last session

- **A paving split should continue into the ground.** At an outline vertex
  where two paved surfaces meet the ground at different heights, the ground
  has one copy, so the earthwork is two-valued there.
  - It accounts for most of the loop box's `step` 391, as centimetre-long
    footpath edges.
  - The fix: two ground copies at the vertex, and a closing triangle along
    the ground edge that divides them.
- **The 26 → 3 steps on the flank roundabout** are measured and bounded.
  `ground::tests::every_drawn_edge_is_welded_or_walled` stays `#[ignore]`d
  until they are 0.
- **`relax.rs` is unwired.** Its next candidate is the pavement between two
  terraces, which is 10 k m² of `split_m2` walls, most taller than 1 m.

## Tools

- **`scripts/world-corpus.sh [BIN] OUTDIR`** runs the specimen corpus and
  the Montreux junction box, one summary file and one `.glb` each.
  `scripts/world-sdiff.py A B` prints only the values that moved. The output
  is byte-deterministic: two runs of one binary diff empty, and anything
  that moves is the change. Take a corpus before the change, then diff.
- **The loop box**, where every number above was taken:
  `./world/target/release/arpentry_world --zone data/zones/montreux --bbox 6.89,46.41,6.96,46.45 --output /tmp/claude/world.glb`,
  about 40 s. `--until arrangement` or `--svg` for the plan alone.
- **Probes.** Write a throwaway `world/examples/zz_*.rs` that runs the
  pipeline to a step and prints what you need, then delete it. Every finding
  in the last session came from one. A number you have not looked at a site
  of is not yet understood.
- **Temporary instrumentation.** Copy the file aside, add `eprintln!`, run,
  then copy it back. Never commit it.
