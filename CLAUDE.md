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

**The GLB carries triangles only unless `--outlines` asks otherwise.** Six
layers are the world — `ground`, `carriageway`, `pavement`, `roadway`,
`deck`, `bore` — and eight are construction lines: the draped centrelines,
the solved profiles, and the ribbon, surface, kerb, fillet, room and facade
contours. Those eight are glTF `LINES`, and **a viewer is not obliged to
draw line topology**: Apple's (Preview, Quick Look, anything on that
pipeline) reads the line index buffer as triangles instead and invents a
long straight shard between every pair of vertices that are neighbours in
the buffer, in the layer's own colour, right across the model. That looks
exactly like a geometry defect and is not one — the file is valid glTF and
Blender draws it correctly. So they are off by default, `--outlines` puts
them back, and the plan view answers the same 2D question anyway. When a
straight line does cross the model *in Blender*, it is real: measure the
edge lengths per node before believing a render.

The run prints one line per step. The output is byte-deterministic, so
`cmp` between two runs over the same inputs is a regression gate; a
difference means something moved. `--terrain 'ramp?grade=0.05'` (or `flat`,
`hill`, `step`, and the three structure rungs `gorge?depth=30&width=40`,
`ridge?height=40&width=120`, `shelf?drop=30&flank=8`) swaps the DEM for a
synthetic ground, and `--segments
net:cross` (or `straight`, `tee`, `overpass`, `underpass`,
`hairpin?angle=20`, `dual?gap=4`, `tee?d=8&hook=5`, `sidewalk?d=6`,
`corner[?split=1]`, `crossing`, `stub?d=0.5`, `driveway?d=6`,
`roundabout`) swaps the parquet for a synthetic
network, where a step's output is an assertion rather than a look.
`--until terrain` stops after a step; the steps so far are `terrain`,
`drape`, `partition`, `reference`, `profile`, `crossing`, `facade`, `ribbon`,
`surface`, `kerb`, `fillet`, `room`, `mesh`, `bench`, `structure`.

**A way is one profile, and a junction can fall in its interior.** Because a
way is whole, another way can end *on it* — at a bridge abutment most often,
which is exactly where a clearance lift has to travel. `crossing::Net` joins
every station at a shared connector for that reason, not just each profile's
two ends; joined at the ends alone, a deck rose and the street meeting it at
the abutment stayed on the ground (`structure abutment` 5.380 m of joint that
could not meet, against 0.000 before). **The base solve still anchors only way
ends**, so `abutment` reads 0.217 rather than 0.000 — the same defect one
level down, and its fix is a pin mid-run in the limiter.

**The terrain writes bridges and bores the source never marked.** A notch the closing
*refuses* — deeper than `NOTCH_FILL_MAX_M` under a line mapped level across
it — is written into the way's span table as a prior (`reference::promote`,
reported as `promoted`), and from there it is an annotation like any other:
the profile chords across it, and §4.5's consequence rule reads a deck off
the result. Nothing in the chain knows what a bridge is. The prior is not
optional and not a shortcut — derived from the heights alone a gorge yields
nothing, because a street is not grade-limited and follows the reference into
the slot exactly. The mirror is gated harder: a refused **crest** becomes a tunnel prior only
for a class whose ladder cannot climb it — grade-limited, and the crest's rise
past that class's deviation box — because a street may climb anything (S9),
and `a_street_climbs_the_ridge_it_is_mapped_over` is the guard on that. On the
ridge specimen a motorway bores through 40 m of mass and `wall_m2` falls
1 792 → 121. **Its approaches still break grade**: the portals land on the
crest's shoulders, which is where the *terrain* says the mass is, not where a
6 % class can reach — placing a portal the road can get to is a design this
model has not made.

**A prior that overlaps a structure the source already mapped extends it**
(`reference::paint`), keeping the mapper's kind and ordinal: the annotation
says there is a bridge here and the refusal says how wide the slot is. That is
the only way a span grows past its annotation — growing on a bare *reach* was
tried and reverted, because a flat-ground underpass whose approach cuttings
are line-buried reads as one majority-fit run and the bore swallowed both
approaches (6.5 m of honest open cutting became 0). Growing past a mapper's
edge needs a reason; the terrain has one and nothing else here does.

Two rules keep it honest: **a promoted deck claims level 0**
(the terrain says a structure is needed, never that it is above anything —
at level 1, two roads bridging one valley became peers and their crossing was
filed as a data error), and **a notch has two rims** (`two_rimmed` — a valley
wider than the window is refused along each flank rather than at its floor,
and those fringes are the sides of a bowl, not slots to span).

**A way is no longer cut at its annotation edges.** The reader converts
Overture's bridge, tunnel and indoor fractions to *arc* and hands the way on
whole, with its spans as an attribute (`world::Way`); the `partition` step is
what cuts them into the ground pieces every surface step builds from and the
pieces off it. It used to cut in the reader, and that made a mapper's split
point a survey point: a piece end is a connector, and a connector is where
the profile pins a height to the ground, so a bridge annotated short of a
gorge lip had its deck pinned to the DEM inside the approach. The cut is now
a step with one caller, and it is where the annotation will hand over to the
solved geometry (`data/plans/spans-are-derived-2026-09-09.md` R1). Today it
cuts exactly what the source said, and reproduces the reader's own piece
list — `plan=2369 spans=181` on the loop box — so nothing downstream can yet
tell that it moved. **The `reference` and `profile` steps still run on
pieces, not ways**, which is why `short` still reads 47.8 %; moving them is
the half of R1 that is not behaviour-neutral.

The `reference` step is the surface every height in the model is solved
against (migration steps 2–3 of
`data/plans/spans-are-derived-2026-09-09.md`): the terrain with its blind
runs bridged, its narrow notches filled and its narrow bumps shaved, plus
the runs the two morphological passes **refused**, which are the terrain's
own bridge and tunnel priors. A DEM is not a ground — it images a culvert as
a slot the road dives through, canopy ripple as crests on the carriageway,
and a viaduct as ground the road is already lying on — and until this step
the world solved against it raw. Its summary line: `short` (pieces shorter
than the 60 m window: a piece is not a corridor, and 47.8 % of the box's are,
which is the argument for the reader keeping whole ways), `blind`/`blind_m`,
`bridged`/`bridge`, `filled`/`fill`, `shaved`/`shave`, `notch`/`crest` and
`off`. **The three passes are measured separately on purpose**: they move the
surface for three different reasons, and a composite number belongs to none
of them — read as one it showed a 13.26 m "shave" against a 4 m budget, which
was the bridging setting a causeway down on its rims. On the loop box:
`blind` 0.16 % / 150 m, `filled` 21.8 % to 14.09 m, `shaved` 9.7 % to 3.96 m,
`notch` 1/43 m, `crest` 12/420 m.

**One reference at a junction.** The conditioning is per axis, so a notch at
a junction could be closed by one way and *refused* by its neighbour, and the
two then stood a whole `NOTCH_FILL_MAX_M` apart at a point they share —
15.103 m on the loop box. `reference::agree` gives every connector one value
before any profile solves (the mean of what the incident axes made of it) and
tapers the correction to nothing over the window that caused it, so the
reference away from the ends is untouched. `junction` reports
`before->after`, and the after must read 0.000.

**The profile solves against it, and `Station` carries both surfaces.**
`ground` is the raw DEM — what the bench owes its earthwork against and what
a departure is measured from — and `reference` is what the limiter aims at
and the deviation box is centred on. So `float` guards the limiter against
the reference and `off` reports the distance from the DEM, which is the
number the departure criterion will threshold; **`off` therefore means
something different than it did before 2026-09-10 and is not comparable
across that change.**

**Solving against it made the road better and the ground worse, and that is
one fact, not two.** On the loop box `grade` fell 0.34 → 0.25 % and `steep`
15.30 → 14.80 %, while `fill` rose 12.476 → 13.319 and `wall_m2` 21 912 →
23 291. The road no longer dives into a notch it was engineered across, so
the bench must build the embankment that was always owed — and past
`MAX_BENCH_FACE_M` the bench does not batter, it walls. Measured, not
inferred: with the blindness mask forced off the box reads `wall_m2` 23 280,
within 0.05 %, so the *closing* did all of it and the bridging did 11 m².
That residue is what `DECK_STANDOFF_M` converts when the partition lands: a
13.3 m fill drawn as a 13.3 m wall is a deck drawn wrong.

Two things in it are not the server's. **The blind mask reads the higher
flank, not the lower**: only a surface proud of the ground on *both* hands is
standing on something, and read against the lower flank — which is what the
server's own guard does inside its bridge trim — the mask fired on 70.6 % of
the box's stations, every contour road above Montreux whose downhill side is
metres below it by construction, and bridging those runs lifted the reference
by 145 m. **And the morphological padding is one station spacing, not one
node.** The erosion has to read the dilation at `arc − r` for every station,
and one pad node supplies that for the first station only; from the second on
the closing lifts the head of every rising axis by up to `r · grade` and
reports it as a filled notch — 1.2 m over the first 30 m of a 5 % ramp, out
of nothing but the edge. `closing_is_the_identity_on_a_ramp` is the check.

**The three structure rungs and `world/src/spans.rs` are a plan, not a
step.** `gorge`, `ridge` and `shelf` put a feature across the way, or level
the ground along it while it falls away beside it, so "is there a bridge
here" has an answer the spec itself gives. `spans.rs` holds eight
`#[ignore]`d checks over them — the specification of the partition step
`data/plans/spans-are-derived-2026-09-09.md` describes, written before the
step. `cargo test -- --ignored` is that plan's to-do list; each failure names
one rule and prints the summary the world reads today. **A check comes off
`#[ignore]` when its rule lands and never before**, and the suite stays
green meanwhile. The 2D plan is
`data/plans/flat-network-2026-09-06.md`; the vertical one (profile →
mesh → bench → structure → crossing) is
`data/plans/surface-leaves-the-plane-2026-09-08.md`. **The order the
steps were written is not the order they run in**: `crossing` re-solves
the profile, so it lands where the profile is read, fourth.

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

**A road may be steep; it may not change how steep it is too fast.** The
class ladder now carries a third number beside the ceiling and the box: a
**vertical curve radius** (`grade::Grade::radius_m`), because the ceiling
bounds the first derivative and nothing bounded the second. Switzerland is
full of 20 % roads and they are fine — what no car can drive is a 20 % road
meeting a flat one inside a metre, and no ceiling forbids that because neither
grade is over the limit. A radius `R` lets the grade change by `1 / R` per
metre, so over a station whose neighbours are `d1` and `d2` away the height
may stand at most `d1·d2/(2R)` off the chord between them, which is what
`profile::bend` clamps. Motorway 4000 m, primary 2000, secondary 1500, street
**100** — the last a drivability floor rather than a comfort figure. A draped
class has none: a stair is a sequence of vertical breaks and bounding them
would be a lie.

**The street's 100 m is the knee, and it was measured.** The *cost* of a
bigger radius is nearly flat (the road stands 0.35 m off the DEM at p90 at
25 m, 0.48 m at 400 m), so the choice is not made on cost. What makes it is
whether the constraint can be met: `kink` — street runs still bent tighter
than allowed — holds near 3 % up to 100 m, then goes 11 % at 200 m and 27 % at
400 m, because smoothing harder would take the road further from its reference
than the deviation box permits. It agrees with the physics from the other
side: 0.3 g at 60 km/h, the fastest thing in the bucket, wants ~96 m.

**The engineered radii are design facts and the box is what cannot pay for
them.** `boxed` (36 % of engineered runs on the box) is a separate counter
for exactly that reason: a street that cannot hold its curve is a road left
undrivable, while a motorway that cannot is eight metres of deviation box
refusing the earthwork a real motorway gets. Lowering those radii only reports
fewer failures — a sixteenth of them still leaves seven runs short.

Two things about it. **The clamp diffuses**, unlike the grade limiter, which
walks both directions and converges geometrically — a broad kink flattens at
about one station per pass, so `BEND_PASSES` is 32 (`kink` reads 39.9 % at
one pass, 15.0 % at eight, 3.0 % at thirty-two). And **a street has no
ceiling but does have a curve**, so `limit`'s early return has to ask about
both; asking about the ceiling alone skipped `bend` for most of the network.
The profile step reports `kink` (runs still bent tighter than their class
allows) and `bend` (the tightest radius held anywhere).

**Where the tags and the geometry disagree, the geometry wins**
(docs/GENERATION.md §4.5). Overture's level, bridge and tunnel tags are
claims a mapper made about a local situation, and on any given feature the
claim may be wrong; the terrain and the network are measurements. Three rules
enforce it, each named where it lives:

- **A dangling deck holds its level; a dangling bore holds the ground**
  (`profile`, `clamped`). A chord with one end anchored and the other
  reaching nothing runs level to the anchor — the named deferral for a deck
  cut by the bbox. For a bore it is not: run level out of a hillside that
  falls away, a tunnel emerges into the air, and the structure step builds a
  viaduct. On the loop box a service road tagged `is_tunnel` end to end ended
  62 m over the ground on **130 m piers**, and asked a motorway to climb 68 m
  out of its way. One clamp fixes it: `lift` 68.19 → **5.16**, `pier`
  130.5 → 82.1, `bores` 83.7 % → 89.9 %.
- **No clearance may spend more than `MAX_CLEARANCE_LIFT_M`** (15 m, the
  server's number and reasoning), and a demand past it is **dropped whole,
  not capped** — spending fifteen of sixty-eight metres leaves the geometry
  wrong *and* distorted. `unstacked` counts the drops, and they do not
  inflate `clearance`, which counts what the model tried and failed to meet.
  It reads 0 on the box now, because the clamp above removed its cause: it is
  a backstop, not a workaround, and `a_demand_the_geometry_contradicts_is_dropped_not_spent`
  is what keeps it honest.
- **A span's own extent is the solved geometry's, not the annotation's** —
  `partition::spans`, and it is now the one span truth: the pieces are cut
  from it *and* the profiles are written back with it, so the surface steps,
  the structure step and the bench all cut the same thing. A span the heights
  bear out is trimmed and grown to the run they imply; a run the annotation
  never had is added. **A span no derived run overlaps is kept whole** —
  absence of evidence is not evidence of absence, and a 25 m bridge over a
  stream is below what a 3.29 m DEM resolves. Degraded for want of evidence
  instead, 37 of the box's structures became earthworks and the ground
  answered with 28 902 m² of wall against 24 643.
- **A bore's ends are elected by its own run** (`partition::bore_bounds`):
  the interior by the line, the ends by the line's crossings where the tube
  fits over the *majority* of the buried run — a real bore grazes only at its
  mouths and those are the portal transition — and pulled back to the tube's
  fit where it does not, which is a surface gallery rather than a bore with
  shallow mouths. The seed is the run of greatest integrated burial, so a
  graze of DEM noise cannot capture the solve from the deep run beside it.
  Three things it is easy to get wrong and each costs a measurement: the fit
  is judged against the **raw ground** (that is what is drawn, and what
  `open`/`cover` measure), it applies to a **proven** bore only (applied to
  what the whole-span guard holds, it degraded eleven of the box's tunnels),
  and it is read from the **annotation's** window with the derived run as a
  floor (clamped to the trim it can only shrink, and a 120 m ridge came out
  with 96 m of tube and both portals buried).

The `crossing` step is the only thing in the model that couples the
height of one way to the height of another. Two carriageway axes whose
**interiors** cross in plan with no connector between them are a grade
separation — Overture cuts a way at every connector, so a junction is
always a meeting of way ends — and the upper must clear the lower by
`ROAD_CLEARANCE_M` (5 m) plus the slab, or, over a bore, by the tunnel's
own `TUNNEL_HEIGHT_M` plus the cover: 6.5 m either way today, because a
tunnel is as high inside as a road is over a road. **What moves is the
one further from the ground**: at a road over a road the bridge lifts, at
a road over a tunnel the bore dips, by the level ordinals alone. The
deficit is spread along the network by a Dijkstra decaying at the class's
ramp grade — a *floor*, never a tent at one node — so 6.5 m of clearance
buys 43 m of approach at a street's 15 %, and the profile re-solves with
the floor added to the ground it solves against. Nothing downstream
learns a rule: on flat ground `net:overpass` degrades in the profile step
(a chord at grade is not a bridge), and after the floor the bench reads
`fill` 6.5 m and the structure reads one deck whose soffit clears the
ground by exactly 5 m; `net:underpass` reads `cut` 6.5 m and one bore
whose crown lies exactly 1.5 m under it. **A chord is charged whole**: a
span is solved as a straight line between its abutments, so the demand
goes to every station of the chain rather than to the crossing, or the
deck would be lifted at its ends and left under the road in the middle.
Its summary line: `crossings`, `same` (two interiors at one level — a
data error, counted and not solved), `orphan` (a demand with no solved
piece on both sides: zero by construction, docs/GENERATION.md §4.5),
`demands`, `lift` (the largest displacement spent), `ramped` (stations
the floor moved), `clearance` (demands still short) and `short`. On the
loop box: 39 crossings, 0 same, 0 orphan, **6 demands**, `lift` 4.61 m,
`ramped` 0.47 %, `clearance` 0/39, in 0.01 s. Walks are not in it (the S
stratum only, and a draped class never solves), and rail, when it
arrives, is senior and enters as a constant. The plan view rings every
crossing green where the clearance was met and red where it was not, over
everything else, with what was asked and what was got in its title.

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
outline no batter is built at all: the bench is *walled* there, `walled`
counts it, and **`wall_m2` of closing face is drawn between the two** — a
step nothing spans is a hole you can see the world through, which is what
invariant 9 forbids. **The kerb gets the same face**: the pavement stands
`KERB_RISE_M` over the road it runs beside, and the two meshes met in plan
and nowhere at all in the vertical, so `kerb_m2` of it is now closed too —
the same defect and the same invariant at a twentieth of the height and a
hundred times the length. It is built off the carriageway mesh's **own rim**
(its boundary edges), so its plan line is a mesh edge and its foot a mesh
vertex, and both rails are read from the meshes rather than computed as
road-plus-rise. `tapered` counts the faces that die away at one end because
the pavement stops there.

**The seam is read, not recomputed, and it is read where both meshes cut
their own edges.** Every outline vertex is a vertex of the room's mesh, so
the ground takes its height from there; but a kerb may run fifty metres
between two vertices of its ring while the profile under it does not run
straight at all, so the ground samples the room's height — and the natural
ground — **at every lattice crossing along the segment**, which is where
both meshes put a vertex anyway. Read at the ring's own corners instead,
the ground interpolated the room's height straight across those fifty
metres and parted company with it in between: on one 400 m road over a
60 m hill that is 60 m of gap and 24 000 m² of it, and `contact` could not
see it because it was read at those same two corners.

**Two of this step's own checks were saying nothing, and now say it.**
`seam` was declared, reported and never incremented — it read `0/…`
because nothing ever counted a miss; it now reads 12 % over the loop box,
which is how often a point of the outline is not a vertex of the room's
mesh. And `contact` compared the ground's height at a point with the room's
at the same point, both from the same closure, which is circular: it is now
**mesh against mesh** — at every vertex of the room's rim that is not a
kerb, does the ground's mesh have a vertex there (`unmet`), and where it
does, how far apart do they stand away from the walls (`contact`)? On the
loop box, 14.0 s: `cut` 9.8 m, `fill` 12.4 m, `step` 0.40 %, `ground`
9.6 M triangles, `seam` 3.2 %, `unmet` 0.93 %, `contact` 0.12 m (a kerb's
rise, and the kerb's own face closes it), `walled` 1.14 %, `wall` 12.2 m,
`wall_m2` 20 171, `kerb_m2` 7 872, `touched` 8.8 %, `off` 17 m.

**Two meshes are the same one at the kernel's grid, not at the weld.**
`unmet` first read 16.65 % and the cause was not the meshing at all: a
mesh welds its own vertices at `mesh::WELD_M`, a micron, and within one
mesh that is right — but the carriageway's regions, the walk's and their
union each come out of the polygon kernel separately, and the kernel snaps
to `poly::GRID_M`, a tenth of a millimetre, a hundred times the weld. A
point that has been through one more boolean than its neighbour lands up
to half a grid away and never welds to it, so at a micron the two meshes
look like strangers along an edge they share. Cross-mesh lookups key at
the kernel's grid and ask the eight cells around as well: `unmet` 16.65 %
→ **0.93 %**, `seam` 12.2 % → 3.2 %, and `kerb_m2` rose 44 % as the kerb
faces that had been tapering to nothing found their pavement. What is left
is a real T-junction — one mesh subdivided a shared edge where the other
did not — and one pre-subdivided outline given to all three meshes is the
fix for that.

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
solid.

**The underside of a deck is the soffit where it clears the ground and the
ground itself where it does not.** That one rule is the abutment block, and
nothing was added for it: a slab `DECK_THICKNESS_M` thick has no soffit out
of the ground until its roadway is that far over it, so every run begins
and ends with a stretch whose slab would otherwise lie *inside* the hill.
Seating the underside there gives a deck the block it lands on, and gives a
run that never clears at all — 23 of the box's 65 — the embankment it
always was, which nothing else builds, since the surface steps read the
ground pieces only. The ground is read across the section, at the axis and
both edges, so the block neither buries its middle in a crown nor floats
its low side on a cross-slope. **Piers** stand a bay of `PIER_SPACING_M`
(45 m) apart under every run whose soffit clears the ground by `PIER_MIN_M`
(6 m): a `PIER_M` (2.5 m) square column from the soffit down to the ground
under its own foot, the bays divided evenly so the last is not a stub. A
foot whose square meets the carriageway is **dropped and counted, never
moved** — moving it is a design and this is a prior — and the bay it leaves
unsupported is the honest picture of what the model knows.

Its summary line: `spans`, `fitted`, `carried`, `decks`, `bores`,
`roadway_m2`, `clear` (the least a slab clears the ground, a guard: ≥ 0),
`buried` (solid under the ground: 0 by construction), `blocks`/`seat`
(stations seated on the ground, and the deepest such seat), `seated` (runs
that got a block), `piers`/`pier`/`skipped`, `cover`/`open` (the crown
against the ground between the portals), `grounded` (runs seated end to
end) and `abutment` (a span's end height against the ground piece it lands
on: 0 on the box, by construction). On the loop box: 174 spans, 116 fitted,
19 carried, 65 decks, 29 bores, `abutment` 0.000, `clear` 0.01, `buried` 0,
`blocks` 64/1381 with `seat` 1.0 — which is `DECK_THICKNESS_M` less
`STRUCTURE_MIN_M` exactly, the deepest a block can ever need to rise — and
75 piers, `skipped` 4. **The tallest pier is 81.3 m and the median 49 m,
and that is the profile's `dangling` chord made visible**: the Viaduc de
Chillon leaves the box on its deck, step 9 runs it level to its one anchor,
and the flank falls away under a deck that does not. A pier is a look, and
this is what a look is for. There are still no portal faces: a bore's tube
ends at its portal.

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
