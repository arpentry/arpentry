# CLAUDE.md

Guidance for Claude Code when working in this repository. Before asking the human for help, use the build, test, and verification commands below to diagnose and fix issues yourself.

## Documentation

Read these docs before making changes to understand the design and conventions:

| Document | What it owns |
|----------|-------------|
| `docs/MOTIVATION.md` | Project motivation and background |
| `docs/SOURCES.md` | The source data: what Overture carries, what the tiler reads, and the measured gap |
| `docs/DESIGN.md` | Design principles (deep modules, pull complexity downward, define errors out of existence) |
| `docs/STYLE.md` | C coding style guide |
| `docs/FORMAT.md` | Tile format specification: geometry model, coordinate space, properties, FlatBuffers schema |
| `docs/VIEWER.md` | Viewer specification: coordinate pipeline, tile management, rendering |
| `docs/CONTROL.md` | Map control specification: camera parameters, input bindings, pan/zoom/rotate, inertia, fly-to |
| `docs/GENERATION.md` | **The vertical world model**: feature strata and authority, the constraint solve, the engineered ground, the invariants |
| `docs/GROUND.md` | The ground imprint and its per-zoom meshes |
| `docs/ROADS.md` | The horizontal road surface: widths, junction areas, markings |
| `docs/TILER.md` | Tiler mechanics: the five stages, the sort key, the `.arpa` archive layout, the modules, the CLI |

Follow `docs/DESIGN.md` principles and `docs/STYLE.md` conventions in all code.

## Building and Testing

First-time setup (creates the `build/` directory):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
```

After any code change, build and run the full test suite to catch regressions:

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

To run a single test executable directly (faster iteration):

```bash
./build/common/test_common
```

Tests use the Unity framework: `setUp`/`tearDown`, `UNITY_BEGIN`/`RUN_TEST`/`UNITY_END` pattern. Test sources live in `common/tests/` and `client/tests/`. (The tiler and tile server are now the Rust reimplementation in `server/`; build and test them with `cargo build --release` / `cargo test` in that directory.)

## Defensive C Coding

Prevent undefined behavior and null pointer dereferences:

- **Always check allocations.** `malloc`, `calloc`, and `arpt_*_create` can return `NULL`. Handle it (return early, `goto cleanup`, or propagate failure).
- **Initialize structs with `{0}`.** Uninitialized fields cause UB. Use `Type var = {0}` or `memset` for heap allocations.
- **Check pointers before use.** FlatBuffers accessors (`_vec`, `_string`, etc.) can return `NULL` for missing optional fields. Always guard before dereferencing.
- **Bounds-check array access.** Validate indices against `vec_len()` or known sizes before indexing.
- **Avoid signed integer overflow.** Use `uint32_t`/`size_t` for sizes and indices. Cast before arithmetic that could overflow.
- **Free in reverse order of creation.** Match every `create` with `free`. Use `goto cleanup` for multi-resource functions to avoid leaks on error paths.
- **No use-after-free.** Set pointers to `NULL` after freeing when they might be checked later.
- **`snprintf` over `sprintf`.** Always use `snprintf` with a size limit. Check the return value for truncation when building URLs or paths.
- **`sizeof(*ptr)` over `sizeof(Type)`.** Keeps allocation size correct if the type changes: `malloc(sizeof(*ptr))`.
- **Use `const` for read-only pointers.** Prevents accidental mutation and documents intent.
- **Cast narrowing explicitly.** When converting `double` → `float`, `size_t` → `uint32_t`, or `int` → `uint8_t`, use an explicit cast to show the narrowing is intentional.

## Key Conventions

These are gotchas not documented elsewhere:

- **FlatCC generates lowercase filenames**: `example_builder.h`, `example_reader.h` (not `Example_builder.h`)
- **FlatCC on newer Clang** needs `-Wno-error=c23-extensions`, `-Wno-error=unused-but-set-variable`, `-Wno-error=implicit-int-conversion` (already configured in root CMakeLists.txt)
- **Generated FlatBuffers headers** go to `${CMAKE_BINARY_DIR}/generated/flatcc/`. The `flatcc_generate` custom target compiles schemas.
- **glfw3webgpu v1.2.0**: the surface function is `glfwGetWGPUSurface()` (not `glfwCreateWindowWGPUSurface`)

## The Fast Development Loop

By default a `--zone` preview still reads the full Switzerland inputs, because bbox
pruning happens at parquet *row-group* granularity — a 25-tile zone still
pulled 13 of 645 row groups of a 954 MB `segment.parquet`, read 165k features
and built 23,611 corridors. The model stage (assemble + solve + ground) is
therefore flat in the size of `--zone`: 51.6 s of a 74 s run. **Shrinking the
bbox does not speed up the loop. Shrinking the input does.**

Cut a zone once, then loop against it:

```bash
# Once per zone (~1.5 s; 2.6 GB of inputs -> 52 MB).
./scripts/cut-zone.sh montreux 6.86,46.40,6.98,46.47

# Every iteration after that.
./scripts/run-overture-ch.sh --zone 6.86,46.40,6.98,46.47 \
    --data data/zones/montreux --screenshot /tmp/claude/preview.png
```

`cut-zone.sh` filters each layer on the `bbox` struct Overture ships (column
statistics, no WKB decoding) into small row groups, and cuts the DEM over the
same bounds. `--data` points the run script at the result and implies
`--skip-download`. `--margin` (default 0.05°, ~5.5 km) sets how much ground
past the zone is cut; the calibration behind that default is in the script's
header. `data/zones/<name>/zone.env` records the zone, the margin and the
source mtimes, so a cut that has fallen behind its source is a diff.

**Two rules follow from how the cut works:**

- **Compare a cut zone's scorecard only against another run over the same
  cut.** The committed baselines were taken over full inputs, which is a
  different population *and* a differently-conditioned solve: the full-input
  run admits corridors that reach past the DEM extract and solve against the
  flat-0 fallback (it reports a 300.65 m clearance shortfall where the cuts
  report ~3 m). Re-cut a baseline per zone, or diff cut against cut.
- **The run summary's global stats are scene-wide, so the cut boundary
  contaminates them even when the measured tiles are fine.** Trust
  the zone proper for anything that must be comparable, not the scene-wide
  line.

### Reading the run summary

Every tiling run ends with per-stage timings and counters. Two lines carry
information no other tool can give you:

```
consistency  junction step max ..., clearance shortfall max ..., N demands dropped
cdt          N tiles lost their breaklines, M fell back from the one mesh
```

The `cdt` line counts tiles the geometric kernel refused. Both fallbacks are
correct (invariant 6: plain, not wrong) but silent everywhere else — the
an archive reader sees a plain lattice either way, so a nonzero count is
invisible downstream. **A tile that lost its breaklines has no imprint creases;
a tile that fell back from the one mesh was built by the pre-S5
construction.** Either way it does not match its neighbours, so check this line
before attributing a seam or a step near that tile to the change you just
made.

## Verifying What the Tiler Produced

**There is no scorecard right now.** `arpentry_verify` (77 metrics over an
emitted archive) and `arpentry_render` (a pixel-diff gate over a corpus of
views) were both removed, along with their committed baselines and the
scenario corpus, to make room for verifying each pipeline step on its own
rather than scoring the finished archive. Nothing has replaced them yet.

What that leaves, and what to use meanwhile:

- **The run summary.** Every tiling run still prints per-stage timings and
  counters, and two of its lines carry information nothing else gives you
  (see "Reading the run summary" above). The `consistency` and `cdt` lines
  are the only automatic statements about geometric quality left.
- **The unit tests.** `cargo test` in `server/` (545 tests) and
  `ctest --test-dir build` (83) still cover the geometry and format kernels.
  They test constructions, not emitted archives.
- **An A/B against a known-good build.** The archive is byte-nondeterministic,
  so compare content, not bytes: build the reference revision in a worktree,
  tile the same cut zone with both binaries, and diff tile by tile with
  `cargo run --release --example adiff -- <a.arpa> <b.arpa>`. Run the
  reference against *itself* first — the same-binary diff is the noise floor
  (on the Montreux cut it is 207 of 866 tiles), and only a count above it
  means something moved.
- **A screenshot, knowing what it is.** `--archive --headless --screenshot`
  is a look, not a measurement. It finds a class of defect once.

**When the new step-verification lands, write the check before the fix.** That
discipline is the one thing worth keeping from the harness that was removed: a
defect that stays screenshot-only is one nobody can tell has come back.

## Verifying Rendering Output

You can visually verify rendering by capturing a screenshot and reading the PNG with the Read tool:

```bash
# Server is the Rust reimplementation; build it with `cargo build --release` in server/.
# A non-archive path (here "tiles") makes it synthesise tiles procedurally.
./server/target/release/arpentry_server tiles style.json 8090 &
SERVER_PID=$!
# angles in degrees, altitude in meters
./build/client/arpentry_client --lon 6.6 --lat 46.5 --alt 50000 --bearing 0 --tilt 0 --screenshot /tmp/test.png
kill $SERVER_PID
# Then use the Read tool on /tmp/test.png to inspect the image
```

The client supports these CLI arguments (native only, not Emscripten):

```
--url <base>          Server base URL (default: http://localhost:8090)
--lon <deg>           Camera longitude in degrees
--lat <deg>           Camera latitude in degrees
--alt <m>             Camera altitude in meters
--bearing <deg>       Camera bearing in degrees
--tilt <deg>          Camera tilt in degrees
--width <px>          Window width (default: 800)
--height <px>         Window height (default: 600)
--archive <path>      Read tiles from a local .arpa instead of a server
--hide-terrain        Draw the tiles without their ground: the network view
--headless            No window, no surface, no display (requires --screenshot)
--screenshot <path>   Capture a PNG after tiles load, then exit
```

With `--screenshot`, the client waits for all visible tiles to load, captures one frame, prints `[SCREENSHOT] saved <path>`, and exits with code 0 on success.

For a capture that is a measurement rather than a look, use `--archive` and `--headless` together — no server, no port, no window:

```bash
# Once per style: write style.arps + models.arpm beside the archive.
./server/target/release/arpentry_server data/overture-ch/preview.arpa style-overture-ch.json --bundle

./build/client/arpentry_client --archive data/overture-ch/preview.arpa \
    --lon 6.9290 --lat 46.4200 --alt 900 --bearing 30 --tilt 50 \
    --headless --screenshot /tmp/claude/cut.png
```

**A served render and an archive render are not always the same image.** The server synthesises a flat sea-level tile for every archive miss so the interactive viewer has terrain everywhere; the archive path reports the miss and falls back to a real ancestor. At 8 km over the Montreux cut that is 66 invented tiles. Compare archive-to-archive, and treat `--archive` as the reference — it draws only what the tiler emitted.

Add `--headless` when the capture is a measurement rather than a look. It needs no window and no display, so it runs over ssh and beside a tiling run, and `--width`/`--height` are then the image size exactly (a windowed capture inherits the display's pixel ratio, so `--width 800` writes 1600 px on a 2× screen). Headless captures are byte-identical across runs — see `docs/VIEWER.md` "Headless capture".

## Asking Why the Asphalt Is There

The scorecard says a surface is wrong; the render says where. Neither says
*which construction put it there*. At the surface zooms a tile carries only the
result of the network synthesis — one unioned mesh per family, every source
stroke deleted — so a disagreement between the plan and the surface it produced
is invisible in the one place it could be seen.

`--plan-lines` (tiler) re-emits the plan-space network as lines, and
`--hide-terrain` (client) removes the ground everything else is drawn against,
so the geometry reads as geometry:

```bash
./server/target/release/arpentry_tiler --output /tmp/claude/net.arpa \
    --plan-lines ...                                        # same args as usual
./server/target/release/arpentry_server /tmp/claude/net.arpa \
    style-network.json --bundle
./build/client/arpentry_client --archive /tmp/claude/net.arpa --hide-terrain \
    --lon 6.9290 --lat 46.4200 --alt 560 --bearing 30 --tilt 50 \
    --headless --screenshot /tmp/claude/net.png              # then Read the PNG
```

`plan_axis_*` is where the model says the surface is. `plan_bound_*` is where it
ends — the boundary of the buffered polygon the union actually consumes, from
the bake's own `pavement::buffer_run`. `plan_edge_*` is the per-segment
cross-section, and is **not** an outline: it steps at every vertex by
construction, so its discontinuity is not a defect. The gap between a
`plan_bound_*` and the drawn rim beside it is the union plus the curb-return
closing — at an intersection that is most of the asphalt. Details in
`docs/TILER.md` "The network view".

**It is a debugging archive, not a map** — the run summary's `plan` line says so.
Serving one to a map style draws three features per source segment that no
style handles. `scripts/run-overture-ch.sh` has no passthrough for the flag, so
this loop means invoking the tiler directly. Any future check must drop every
`plan_*` class on the way in: this emission poisoned a surface metric exactly
that way once, because the metric filtered on `level` rather than class.

## The World Crate

`world/` builds a tile-free 3D world for one bounding box, one verifiable
step at a time, and writes it as a binary glTF. It borrows only the source
readers from `server/`; the model is rebuilt from the raw sources, step by
step, each step with its own tests on synthetic ground. Nothing in it knows
what a tile is, and that must stay true: the tiler is meant to become one
more caller of it.

```bash
cargo test  --manifest-path world/Cargo.toml
cargo build --release --manifest-path world/Cargo.toml

# The loop box: the lake, the shore, the town and the mountain flank to 1639 m.
./world/target/release/arpentry_world --zone data/zones/montreux \
    --bbox 6.89,46.41,6.96,46.45 --output /tmp/claude/world.glb
```

The run prints one line per step. The output is byte-deterministic, so
`cmp` between two runs over the same inputs is a regression gate; a
difference means something moved. `--terrain 'ramp?grade=0.05'` (or `flat`,
`hill`, `step`) swaps the DEM for a synthetic ground, and `--segments
net:cross` (or `straight`, `tee`, `hairpin?angle=20`, `dual?gap=4`,
`tee?d=8&hook=5`, `sidewalk?d=6`, `corner[?split=1]`, `crossing`, `stub?d=0.5`,
`driveway?d=6`, `roundabout`) swaps the parquet for a synthetic
network, where a step's output is an assertion rather than a look.
`--until terrain` stops after a step; the steps so far are `terrain`,
`drape`, `profile`, `facade`, `ribbon`, `surface`, `kerb`, `fillet`,
`room`, `mesh`, `bench`, `structure`. The 2D plan is `data/plans/flat-network-2026-09-06.md`; the vertical
one (profile → mesh → bench → structure → crossing) is
`data/plans/surface-leaves-the-plane-2026-09-08.md`.

The reader cuts every way at its bridge, tunnel and indoor span boundaries
and keeps every piece with its `kind`: the ground pieces are `Roads.plan`,
which every surface step builds from, and the rest are `Roads.spans`. The
`profile` step gives every carriageway piece a height along its axis:
connectors shared, engineered classes grade-limited inside a deviation box,
streets on the ground exactly (a street mapped at 20 % climbs 20 %), and a
mapped span a straight chord between its anchors. Its summary line is the
first vertical check: `grade` (engineered pairs over the ceiling), `steep`
(street pairs over 15 %, information), `float` (0 by construction), `step`
(the largest height disagreement at a connector, 0 by construction),
`decks`/`bores` (stations of mapped spans that stand off the ground by
0.5 m), `degraded` (spans that never leave the ground), `dangling` (spans
cut by the clip, held level to their anchor) and `unanchored`. On the loop
box the Viaduc de Chillon is one 1.6 km deck and the Glion tunnel a 1.4 km
bore. `net:straight?span=0.3,0.7[&kind=tunnel]` is the specimen; the plan
view draws cut in blue and fill in red along the axis, a deck dashed and a
bore dotted; the GLB gains a `profile` line node at the solved heights.

The `mesh` step triangulates the carriageway and the pavement **conforming
to the terrain lattice**: each region is ear-clipped and every ear cut by
the lattice's columns, rows and cell diagonals, so every triangle lies in
one terrain triangle and, with its vertices at `height_at`, on the ground
to the ulp — the drape guarantee for areas. The bench step then moves
them off it; on a flat ground the two stay coplanar and z-fight in any
3D viewer, so hide the terrain to see them.
Its summary line: `lost_m2` (region area the triangles miss), `off_ground`
(the largest height a triangle's centroid stands off the terrain),
`seam` (the mesh's one-sided edge length less the regions' perimeter: a
crack or an overlap shows here, and it reads 6e-9 m on the box),
`washed`/`lossy` (regions the ear clipper misread; a washed one was read
right from the kernel's union of it), `slivers`, `welded`. The rings are
cleaned at the kernel's lattice (`COLLINEAR_M`, 0.1 mm) before clipping:
the kernel leaves straight edges zigzagging by a few hundredths of a
millimetre and spikes a lattice cell wide, and the clipper turns both into
T-junctions. The plan view draws the wireframe in windows under 100 m.

The `bench` step lifts the room off the ground onto the height the
profile solved: a point of the room takes the profile height at the
perpendicular foot on the nearest **ground** carriageway axis (a deck's
chord is the structure step's), and the pavement stands `KERB_RISE_M`
(0.12 m) above it. The road is level crosswise, so a 5.5 m residential
along the contour of a 30 % slope is cut 0.825 m at its uphill kerb and
filled 0.825 m at its downhill one, exactly. The cross-section is level
for `ROOM_REACH_M` (6 m) past the asphalt — the room step's own wall
reach — and past that the walk comes down a face at `EARTHWORK_BATTER`
(1 in 2.5) and stops where it meets the ground. A band standing further
than one face (`MAX_BENCH_FACE_M`, 3 m) from the road beside it is not
that road's pavement and drapes: **without that test the loop box read
225 m of cut**, the field having carried a road's height half a
kilometre up the flank, because a face into a mountain steeper than
1 in 2.5 never daylights. Its summary line: `lifted`/`battered`/`draped`
(where the room's vertices stand), `walled` (vertices more than one face
off the ground — where the ground's answer must be a wall, not a
batter), `cut`/`fill` (the earthwork the ground still owes), and `step`
(mesh edges the field is discontinuous across, over `KERB_RISE_M` and
steeper than `STEP_GRADE`) with `worst`. The plan view marks every step
with a purple dot in any window under 2 km, like the room's bare kerb
stations.

**And the ground answers.** The terrain is re-triangulated over
`rect − room` on the same lattice by the same mesher, so **the ground
stops at the kerb**: no triangle of it lies under the asphalt, which is
where every artefact of a ground drawn beneath an opaque surface lives
(`data/plans/terrain-hole-plan.md`). Outside the room it is the room's own
height at the outline, a face at `EARTHWORK_BATTER` out of it stopping
exactly where it meets the natural ground, and the natural ground beyond.
A face is at most `MAX_BENCH_FACE_M` tall, so it runs at most 7.5 m, and
where the room stands further than one face from the ground at its own
outline no batter is built at all: the bench is *walled* there and
`walled` counts it. **The seam is read, not recomputed** — every outline
vertex is a vertex of the room's own mesh, so the ground takes its height
from there. On the loop box, 11.7 s: `cut` 9.8 m, `fill` 12.4 m, `step`
0.40 %, `ground` 9.6 M triangles, `seam` 0/94 575, `contact` 2.5e-7 m,
`walled` 1.45 %, `wall` 12.2 m, `touched` 8.9 % of lattice vertices,
`off` 17 m.

The `structure` step builds what the solved profile implies, and nothing
else: a mapped bridge whose chord never left the ground gets no deck.
**The roadway comes first** — the surface steps read the ground pieces
only, so until this step a way's bridge and tunnel spans carried no paving
at all (74 469 m² on the loop box). Every span piece is swept at its
solved height across its own width, so the road is continuous over the
Viaduc de Chillon and through the Glion bores. **The solid is only what is
underneath**: over a deck run a soffit `DECK_THICKNESS_M` (1.5 m) below the
roadway with its sides and end faces, over a bore run a crown
`TUNNEL_HEIGHT_M` (5 m) above it with its walls, open at the portals — the
roadway is the deck's top and the bore's floor, once, so no two surfaces
of the step are coplanar. A **pedestrian span is fitted, not solved**
(most of the box's spans are): a chord between the ground at its own two
ends, no ceiling and no box, and a footbridge's own `WALK_DECK_M` (0.4 m)
deck rather than a road bridge's. **Unless the road already carries it** —
a separated sidewalk mapped as its own bridge is one structure with the
road, so a walk span within the room's reach of a road's span all along is
*carried*: its height is that road's plus the kerb's rise and it builds no
solid. Its summary line: `spans`, `fitted`, `carried`, `decks`, `bores`,
`roadway_m2`, `clear`/`buried` (the soffit against the ground *between*
the abutments), `cover`/`open` (the crown against the ground between the
portals), `grounded` (runs whose face never left the ground at all) and
`abutment` (a span's end height against the ground piece it lands on: 0 on
the box, by construction). On the loop box: 174 spans, 116 fitted, 19
carried, 65 decks, 29 bores, `abutment` 0.000, and `grounded` 25 —
**a quarter of the box's structure runs are shallower than the deck they
would need**, because `STRUCTURE_MIN_M` makes a piece a deck at 0.5 m off
the ground while `DECK_THICKNESS_M` gives it a 1.5 m slab. That is what
the abutment block, which the plan defers, is for. There are no piers, no
abutment blocks and no portal faces: a deck ends in the air at its soffit
and a bore's tube ends at its portal.

**What is still missing.** Neither the toe nor the wall is a breakline, so
a lattice triangle may straddle one and stand off the engineered ground
between its vertices — that is `off`, and 17 m of it is a triangle
spanning the tallest wall and the batter beside it. Nothing re-drapes yet:
the free lines and bands still sample the raw terrain, so a footpath
leaving a street does not run up the batter. And `height_at` is still the
terrain's; the structure step is where that has to change.

The `facade` step reads the zone's `building.parquet` (when it is there) and
every surface step after it keeps out of the footprints: the pavement stops
at the walls, the asphalt at the *closed* walls (notches and gaps between
houses under 3 m are pockets it does not enter, so its edge does not zigzag
with the building outline). A way whose axis runs through a footprint or a
pocket keeps a corridor of at most 4 m (`facade::PASSAGE_M`) so the network
is not cut in two. The `room` step then paves the kerbs that run along facades: where a wall
face stands within 6 m (`room::WALL_REACH_M`) over at least 6 m of kerb,
bridged across gaps under 10 m, a 2 m band runs along the kerb and rungs
reach from it to the wall — the strip along a house front, the pocket a
kerb return leaves at a house corner, a notch, an alley mouth. A corner
that merely points at the road gets nothing. The probe stops at mapped
pavement too, so a pocket between a kerb return and a sidewalk wrapping the
corner, or a break in a sidewalk under 25 m, is paved at the sidewalk's own
width. A hole in what is built, asphalt, pavement and
buildings together, under 500 m² (`room::ISLAND_M2`) that borders asphalt
or pavement and lies wholly within 10 m (`room::POCKET_REACH_M`) of the
asphalt is paved too: a roundabout's centre, a traffic island, the pocket
in a junction corner between a kerb, a footway and a house. A hole walled
all round is a courtyard, and one that reaches farther from the asphalt is
a lawn; both stay ground. The plan view marks every kerb
station `kerb_gap` still counts with a red dot in any window under 2 km,
so a gap is found by its marker.
`--buildings none` runs without buildings; a synthetic network has none
unless `--buildings house:beside?d=2` (a house `d` m off the axis, `notch=`
for a notch in its facade), `house:across?rot=30` (one the way passes
through), `house:row?gap=2` (two along the road) or `house:pair?gap=3` (two
facing across it) says so. The `kerb_gap` and `wall_gap` checks know the
walls: a kerb station with a facade outside it is walled, not bare.

To look at the world, write the plan view. It is one SVG group per step,
every stroke width a width in metres, a function of the world alone so
`diff` says what moved:

```bash
./world/target/release/arpentry_world --zone data/zones/montreux \
    --bbox 6.89,46.41,6.96,46.45 --terrain flat --spacing 50 \
    --svg /tmp/claude/plan.svg --view -1174,234,-974,434   # metres, x0,y0,x1,y1
rsvg-convert -w 2000 /tmp/claude/plan.svg -o /tmp/claude/plan.png   # then Read
```

`--view` only changes the window (the clip still runs at the bbox), so a
200 m window at 2000 px is 10 px per metre, enough to see a kerb. On a flat
synthetic ground the whole run is under a second.
