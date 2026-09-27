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
