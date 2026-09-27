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

**Next: review step 5, the plan chain from explicit geometry.** The hand-over
is `docs/plans/plan-chain-from-legs.md` — read it first; it has the
baseline counters, the first slice (junction polygons, measured and not
wired) and the tools (`scripts/world-corpus.sh`, `scripts/world-sdiff.py`).

`world/` builds a tile-free 3D world for one bounding box, one verifiable
step at a time, and writes it as a binary glTF. It borrows only the source
readers from `server/`; the model is rebuilt from the raw sources, step by
step, each step with its own tests on synthetic ground. Nothing in it knows
what a tile is, and that must stay true: the tiler is meant to become one
more caller of it.

**A step is a function of what it reads, and nothing else.** Every step is
`fn run(inputs…) -> (Layer, Summary)` over the layers it needs — no step
takes the world, none can reach a layer it did not name, and **none writes
to one**. `World` is the record the layers land in, read only by the two
renderers, which draw whatever has been built. All the wiring is
`world/src/pipeline.rs`: the order (`Step::ALL`), which layer feeds which
step, the three sources (`Sources` — only the terrain, the drape and the
facade read anything outside), and the one place that unwraps a layer and
can therefore assert its predecessor ran. **Adding a dependency between two
steps is a diff in that file**, which is the whole point: it used to be an
`expect("the drape step runs first")` inside the step, invisible from
outside.

Three things that rule cost, each one a defect it had let stand:

- **Three steps took `&mut` and rewrote their predecessors.** `reference`
  promoted the terrain's bridge and tunnel priors onto the ways; `crossing`
  replaced the profiles; `partition` rewrote every way's span table, both
  piece lists, every profile's table and every station's verdict — while
  returning nothing but a `Summary`, which made the most important step in
  the pipeline the one whose signature said least. Between them a way's span
  table was a field three steps wrote to and no signature admitted. They
  return those tables now and the `match` arms install them.
- **The paved surface had no single answer.** It is re-cut four times
  (`surface`, `kerb`, `fillet`, `room`), and "the latest" was resolved at
  runtime by a `World::walk()` over four layers, then by a literal in the
  pipeline assembled field by field — still with a branch in it, because
  `room` itself reads the paving before it has run. All four steps now hand
  back a whole `world::Surface`: what they changed and what they passed
  through. The latest paving is the last layer that laid any, and
  `&room(world).surface` is a lookup like every other. `Paving<'a>` is gone.
- **Which way a profile belongs to was re-derived four times.** `Profiles`
  and `Reference` are subsets of `Roads::ways` in their order, and every
  step holding one recovered the way by re-running the selection predicate
  and inverting it into a `HashMap`, with an `assert_eq!` standing in for
  the type that should have said it. `Profile::way` and
  `reference::Axis::way` record it; `reference::solving_of` is now asked
  once, where the reference is built.

**The order is `Step::ALL`'s and is written nowhere else.** The step modules
used to open with "Step 7: …" in their headers and eleven of the seventeen
had drifted — `crossing` said 13 and runs 5th, `partition` said 3 and runs
6th. The ordinals are gone; a module header says what the step makes.

The specimen ladders live there too (`pipeline::tests::built`), named as
step lists: `upto(Step::Bench)` is the whole prefix, `plan(Step::Ribbon)`
leaves out the three vertical steps a flat specimen has nothing for. They
had been written out by hand in eight test modules and had drifted — the
mesh's ladder still skips `crossing` where the bench's does not, and that
divergence is now visible in one line rather than buried in a copied
sequence.

```bash
cargo test  --manifest-path world/Cargo.toml
cargo build --release --manifest-path world/Cargo.toml

# The loop box: the lake, the shore, the town and the mountain flank to 1639 m.
./world/target/release/arpentry_world --zone data/zones/montreux \
    --bbox 6.89,46.41,6.96,46.45 --output /tmp/claude/world.glb
```

**The GLB carries triangles only unless `--outlines` asks otherwise.** The
world is `ground`, `wall`, `kerb` (every edge face of the edge rule, the
ballast's included), `carriageway`, `pavement`, `ballast`, `roadway` (a
bore's floor and a footbridge's paving — a road's or a railway's deck is
part of `carriageway` or `ballast`), `track` (a railway bore's floor),
`deck`, `bore`, `building` and `roof`, and eight layers are construction lines: the draped centrelines,
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
difference means something moved. **So is the summary itself** — every
metric of every step reproduces exactly, so `diff` between two runs' lines
is the same gate one level up, and cheaper: it needs no `--output`, and it
says which step moved rather than only that something did. (It did not use
to be. The mesh step's `seam` summed the one-sided edges in a `HashMap`'s
iteration order, which `RandomState` reseeds per process, so one binary on
one input reported 2.6e-9, 3.2e-9 and 8.6e-9 on three runs while the GLB
stayed byte-identical.) `--terrain 'ramp?grade=0.05'` (or `flat`,
`hill`, `step`, and the three structure rungs `gorge?depth=30&width=40`,
`ridge?height=40&width=120`, `shelf?drop=30&flank=8`) swaps the DEM for a
synthetic ground, and `--segments
net:cross` (or `straight`, `tee`, `overpass`, `underpass`,
`hairpin?angle=20`, `dual?gap=4`, `tee?d=8&hook=5`, `sidewalk?d=6`,
`corner[?split=1]`, `crossing`, `stub?d=0.5`, `driveway?d=6`,
`roundabout`, `level[?rail=narrow_gauge]`, and `leg=CLASS` on `overpass` and
`underpass` for a rail bridge or a rail bore) swaps the parquet for a synthetic
network, where a step's output is an assertion rather than a look.
**The world crate has a plan of its own open**:
`data/plans/one-ground-2026-09-16.md`, with its checks in `world/src/ground.rs`
in the shape `spans.rs` and `junction.rs` use — `#[ignore]`d against today's
tree, so `cargo test -- --ignored` is the to-do list. Its rule is *one
arrangement, one height per vertex, a step is an edge you declare; and the DEM
is a measurement, so a standard is a floor on what it cannot see and never a
correction to what it can*. Step 0 (`dem_residual`) has landed and answered the
question the plan opened with; §1.2 is what it found.

`--until terrain` stops after a step; the eighteen, in order, are
`terrain`, `drape`, `reference`, `profile`, `crossing`, `partition`,
`facade`, `ribbon`, `surface`, `kerb`, `fillet`, `room`, `sheet`,
`arrangement`, `mesh`, `bench`, `structure`, `building`.

The `arrangement` step is **step 1 of
`data/plans/one-ground-2026-09-16.md`, landed and wired** — the mesh step
triangulates its faces as one mesh (see "The one mesh is wired" below): it cuts
the rect into faces along every material boundary at once
(`poly::slice`, one pass) and tags each face by a single interior point
(`poly::inside`), so the ground and the paving beside it share their
boundary instead of each computing it. Its line reports `unshared` — face
vertices off the rect's edge that only one face carries, which is the
defect, and reads **0** on every specimen — and `closure`, the partition
drift as a multiple of what snapping the rect's own corners to the lattice
can explain (1.0 is that bound; it reads 0.063). `unprobed` counts faces
too thin to name a point inside; they are taken as ground, and on
`house:across` there are 4 of them totalling **0.000 m²** — slivers where a
passage corridor meets a wall.

`poly::slice` costs 0.01–0.03 s at 7 872 rings — 20 to 100× less than the
`union_all` already in the chain — so the cost is the tagging, which is
indexed.

**A partition has one face per point, and a grade separation has two
surfaces there** (`Arrangement::decks`). Where a sheet's span paving lies
over another sheet's *ground* paving, the partition's face is the ground's:
it cuts the terrain and meets its own kerbs. The deck gets a second face of
the same shape, from the same slice, so it still welds to the rest of its
sheet (`mesh::by_sheet` reads `layered`). Before this, the face went to
whichever sheet the index named first. At the Viaduc de Chillon that was the
deck, so the streets under it showed bare terrain. On `net:overpass` it was
the street, so the deck had a hole over it. Three rules go with it, each
fixing a defect that was visible at Chillon:

- **An apart sheet takes its approaches' ground axes, not its parent
  sheet's** (`sheet.rs`). It used to take all 973 profiles of the town's
  connected paving, including the streets it flies over. `bench::by_sheet`
  believes a chord only where it is no farther in plan than the nearest
  ground axis, so over every street under the deck the asphalt hung 44 m
  down in a curtain (bench `worst` 46.632 → 11.846). An approach is a ground
  piece sharing a vertex with the span. An apart deck also gives up the
  approach's ground paving under its round cap, so the two do not lie
  coplanar at the abutment.
- **The edge rule draws no face taller than the slab next to a span face**
  (`edge_faces`). The partition is flat, so a deck's rim and the sidewalk of
  the street below share edges in plan. The slab closes a deck's side, and
  below the slab is air. `kerb_m2` 19 268 → 10 850 on the loop box, of which
  8 082 m² were Chillon's curtains (up to 63 m) and the rest the same case at
  other overbridges and rail crossings, 5–12 m tall. The rule reads *either*
  face spanned, because the street's own sidewalk within 6 m of a deck is
  claimed as walk the deck carries.
- `arrangement` reports `decks`/`deck_m2` (49 / 1 414 m² on the loop box).
  `world/examples/curtain_probe.rs` lists the surfaces over any point, and
  with `decks` the height gap at every deck face. A gap near 0 would be two
  coplanar sheets; the box has three slivers under 0.2 m² in total.

**`net:overpass` at its default 200 m does not separate the two roads.**
Both ways are densified every 4 m and so both have a station exactly at the
crossing. `crossing::Net` joins stations by position, reads that point as a
shared connector, and spreads the leg's floor into the street. The result is
two decks and `clearance` short by 5.38 m, and it is the same at `b74a8b2`.
`net:overpass?len=201` is the honest specimen. The rule that joins by
position is fragile and is not fixed.

**`mesh` and `bench` read it** (step 1b): the mesher triangulates the faces
of each material, grouped by sheet; the bench takes the hole and the ground
from the faces rather than from `union_of([on_ground, walk − spanned,
galleries])` and `rect − outline`. The two regions were measured against
each other first and agreed to **0.000 m²** on every specimen, so what
changed is the boundary and not the geometry. Over the corpus `bench seam`
went **285 → 40 misses, 9 specimens → 1**; every specimen reads exactly 0
except `house:across`. `unmet` is 0 throughout.

**What is left is buildings, and it is four slivers.** `house:across` — a
way passing through a house, so `facade.passages` cuts a corridor — is the
only specimen in the corpus with `unprobed > 0`: four faces of 2.2e-7 to
4.4e-5 m² where the corridor's edge crosses a wall a lattice step off it
(`ARPENTRY_SLIVERS=1` prints them). A degenerate face called *ground* while
it sits inside paving punches a ring into the hole's outline that no paved
mesh has a vertex for, and `dense` subdivides that ring at every lattice
crossing — four slivers, forty misses. So a face too thin to probe adopts
the material of the neighbour it shares the most **vertices** with
(`arrangement::adopt`), and the union absorbs it: corpus `seam` **285 → 50
→ 20**, `house:across` alone.

**And `poly::slice` must be pinned like every other boolean.**
`slice_by_fixed_scale` takes a scale but builds its adapter from the *input's
own bounds*, so its origin moves with the data and its output floats land
between `poly::overlay`'s. The faces agreed among themselves — they come from
one slice — so `unshared` read 0 throughout and nothing showed it; what
showed it was unioning the faces and finding the union had **invented all 27
of its vertices**, not one of them a face vertex. `slice` goes through
`FloatStringOverlay::with_adapter` on the same pinned rect now, and the union
invents none. This was latent everywhere a sliced result met an overlaid one,
and it is the reason to distrust any new kernel entry point that does not
take the adapter.

**Three things were measured on the way and are not it.** Adding both
facade masks as cuts: `seam` 25 → 20 but `unmet` 4 → 8. Matching the
neighbour by shared *edges* rather than vertices, counted or weighted by
length: 25 either way — a sliver's neighbour has usually subdivided the
edge they share, which is the same hairline that made the sliver. And
preferring a neighbour that **cuts**, which is the rule the failure mode
argues for: also 25. That last one is the useful finding — the twenty that
remain are not a sliver landing on the wrong side, so a fifth tie-break
will not move them. They have not yet been read directly, the way the four
slivers were, and until they are this is calibration rather than a rule.

Two rules the rewire established, each bought with a measurement. **The
walk's near/far split is a cut, not a boolean** — found with
`dilate`/`intersect` it put vertices on the pavement no other mesh had — and
the ring is clipped to the walk, because cut over the whole rect it splits
the ground where it means nothing and took `cut` 4.255 → 4.755 m on a 50 %
ramp. **And `bench`'s `outline` is unioned while the faces are not**: the
faces are what the meshes are built from and must stay apart, but `outline`
is only walked for its edges, and the edge between two adjacent paved faces
is not boundary — walked as faces it put 10 interior edges into `seam` on
`house:row` out of nothing.

**The step also reports `edges` and `dangling`** — the adjacency
`data/plans/one-ground-2026-09-16.md` step 2 needs, and whether it exists.
`dangling` counts segments carried by one face alone away from the rect's
border, and reads **96 over 8 of the 41 specimens**. They are real
T-junctions: on `net:level` the carriageway's edge along `y = 2.75` is one
200 m segment while the ground's side is three — ground, ballast, ground —
because the railway cuts the ground there and nothing cuts the road.
`poly::conform` subdivides every ring at every vertex lying on it and takes
that to 3 — **landed**, `seam` cost accepted (corpus 20 → 65).

**The edge rule is `bench::edge_faces`** (§3.3): one function over the
subdivision's edges — welded where the two faces answer with one height,
split where they do not, and then the quad is drawn — replacing `kerb`,
`rail_face` and `walk_face`, each of which walked one mesh's rim and looked
the other side up. `wall` stays, because it sweeps the room's outline
against the *ground*, the one boundary whose far side is not a face of the
paving.

**The edge rule alone did not zero `step` — and step 2 did not need the
field first.** `step` counted triangle edges *inside* one lifted mesh, 69 %
of them away from any arrangement edge, and the plan read that as needing
the arrangement cut where the height field steps (§3.2 before §3.3). The
one mesh gave a cheaper route: declare the step on the mesh's own edges.
See "Step 2 is built" below.

**The solve is `world/src/relax.rs`, built and not wired** (plan step 3a) —
and the ground went the residual way without it; see "Step 3 is built" below.
§3.2's energy `Σ w(z−dem)² + Σ‖∇z−∇dem‖²` is, in terms of the residual
`e = z − dem`, exactly `eᵀ(W + L)e` — a damped Laplacian on the residual with
`e` pinned at the paved vertices. So **ground no pin reaches is the DEM to the
bit** (`e = 0` solves it there), which is what makes `FIELD_LIMIT_M`, the
taper and the handover unnecessary rather than merely replaced; and the batter
is `e` falling to nothing, shaped by the energy instead of by
`EARTHWORK_BATTER`. Conjugate gradients, four properties checked, including
that the decay on a chain matches the closed-form root of
`r² − (2+w)r + 1 = 0` and that a symmetric pin on a grid gives a symmetric
field — which the nearest-axis rule never managed at a junction. `WEIGHT` is
chosen rather than measured: it sets a *decay* where the old constant set a
*slope*, and the two do not convert.

Wiring it needed one mesh over the whole rect, and that has landed (below):
the relax can now be given the one mesh's own vertex graph, which is also
what unblocks step 2's remaining 70 %.

**`wall` can mostly retire into the edge rule, but not the portals.** Given
the ground mesh as its height source `edge_faces` draws the paving|ground
face itself: on `step?rise=10` it is **2055 m²** against `wall`'s **2030**,
the same face within 1.2 %, and removing `wall` breaks one test and only
because the geometry changes field. What it does *not* absorb is the portal
headwall and footing — `ridge` falls 737 → 660 m² and a tunnelled `gorge`
421 → 390 — because that closes the ground onto the **tube's section**, which
is not a paving|ground edge and has no arrangement face. So `wall` shrinks to
the mouths rather than going, unless the tube's footprint becomes a face
(`Face::gallery` is half of that).

**Shrinking it was tried and reverted.** Restricting `wall` to its portal
branch cleared the cliff's double-draw and left `gorge` alone, but `ridge`
fell 737 → 665 m²: `wall` sweeps the outline **densified at lattice
crossings** while `edge_faces` emits one quad per arrangement edge, and over
varying ground a chord misses area and leaves a gap per cell. Subdividing the
edge rule to match moved three specimens by hundreds of m² with no
predictable sign (`gorge` 390 → **31**, which is lookup failure, not
geometry), so all of it went back. The swap needs the rule to read its rails
*by position on the mesh* rather than by a lookup at a recomputed crossing —
which is what meshing once gives, so 3b comes first. Not committed: the
ground is still `wall`'s and the two do not double-draw.

**The one mesh is wired (2026-09-27).** The `mesh` step triangulates every
face of the arrangement, both layers, in one `mesh::tagged` call, so the
paving and the ground share every boundary vertex **by index**; `Mesh` is
that one `Tri` and the face each triangle came from, and `walk_split` and
the per-vertex sheet arrays are gone. The bench copies each vertex once per
surface that reaches it (`bench::Copies`: the ground, each carriageway and
ballast sheet, the near and far pavement), lifts the copies by the rules it
always had, and reads every seam by index: the outline is the one mesh's
own edges between a face that cuts and one that does not, `Ground` is built
from those edges, and `wall` and the edge rule draw one quad per mesh edge.
The eight-cell position lookup, `meet`, `cut_seam`, `rim`, `dense` and the
second ground triangulation are gone. On the loop box `seam` **3.70 % → 0**,
`unmet` **1.28 % → 42 edges of 884 k**, `contact` **2.85 → 0.00**; every
synthetic specimen reads all three at exactly zero, and
`ground::tests::the_ground_and_the_surface_share_their_boundary` is live.
The paved heights did not move — `lifted`/`battered`/`draped`/`cut`/`fill`/
`step`/`worst` were identical on every specimen at the first switch-over.

**Meshing the faces together found cracks that meshing them apart hid**,
and the mesh step's `crack` (the partition's one-sided edges off the rect's
border — replacing the old `seam`, which the arrangement had invalidated)
is how. Three causes, each fixed where it lives:

- **Rings were cleaned one at a time** (`mesh::cleaned`): a vertex that is a
  corner of one face and collinear in its neighbour went from one ring only.
  `mesh::cleaned_together` decides per vertex over every ring that carries
  it, first dropping any face of no area (a slice needle `A, B, C, B` must not
  veto its neighbours) and collapsing a ring's doubled-back `A, B, A`. That
  alone took the Montreux junction box from **13 km** of crack to 75 m.
- **A degenerate ear left a T-junction** (`mesh::close_t_junctions`):
  `earcutr` bridges a hole along the line its edge runs on and cuts a flat
  ear, and the far-side triangle then skips the ear's middle corner.
- **Twin corners a grid step apart** (`mesh::weld_open`): where the slice
  leaves a needle, two neighbours start one edge from two corners 2e-4 m
  apart and cut it at the lattice ~1e-5 m apart all along. Merged at
  `GRID_M`, **open edges only**.

**And two more, found by the loop box's own cracks** (2026-09-27): the slice
leaves **twin corners** a grid step or two apart where cut lines nearly
coincide, and neither `conform` (which splits a segment at a vertex *on* it,
while the neighbour's edge ends at the twin) nor the mesher's micron weld
could pair them — `poly::snap_twins` merges them across every face before
`conform`, and the arrangement's interior T-junctions went 40 → **0**. And a
lattice crossing the far side computed off such a twin lies ~1e-5 m off this
side's edge, so `close_t_junctions` now takes a vertex as on an edge within
the kernel's grid rather than the weld. Loop box `crack` 200 → **21 m**, over
45 faces — zero-area single-triangle slivers and a few edges whose two sides
genuinely disagree by decimetres, which is the plan chain's to stop
producing (review step 5). `unmet` 0; the junction box's `dangling` 50 → 0
(its counter had also been counting the rect's own snapped border).

**`kerb_m2` rose 10 850 → 15 023 on the loop box**, and it is closure that was
missing rather than curtains: 8 017 m² of kerbs (0.12 m over ~67 km),
6 319 m² between the near and far pavement — the designed retaining face
where the walk past the room's reach drapes — and 40 m² between two
carriageway sheets. The old rule read both sides by position and dropped any
quad with a missed end. The edge rule now reports `kerb_max` (the tallest
face) and `sheet_m2` so a curtain has a number to show up in.

**Step 2 is built (2026-09-27): every drawn edge is welded or walled.**
Measured first, and the measurement moved the target: of the loop box's
18 408 `step` edges, **10 028 were the raw DEM's own steepness** under a
draped pavement (both ends at the natural ground), ~8 400 were the lift's
case function switching between two vertices of one triangle, and of those
only ~1 400 were the carriageway's nearest axis — the pavement's nearest
road (3 491) and its face/drape threshold (2 757) were most of it. So the
fix is general rather than per material:

- **A triangle takes one rule** (`bench::Rule`: chord or ground field, the
  axis, the `PART_M` stretch of it, face or drape), chosen at its centroid,
  and all three corners are answered by it (`Lift::height`, `Field::on_axis`).
- **A vertex two rules answer is two copies**, keyed `(Surface, Rule)`:
  welded at their mean within `KERB_RISE_M` (`Copies::weld`), split
  otherwise, and the edge rule draws the face across the split (`split_m2`).
- **Two things made the rules agree where they should.** A junction's legs
  weigh against the *nearest* leg's distance, not the answering one's (with a
  forced axis the blend was asymmetric and two legs of one junction split at
  their shared corners — caught by `a_junction_on_a_slope_is_one_surface`).
  And a foot on a polyline axis blends with the neighbouring segment's
  *interior* foot (`Field::along`, `SEGMENT_BLEND_M`): on the inside of a bend
  the nearest foot jumps a station's spacing along the axis, a step of metres
  on a steep street. Blended with the neighbour's shared *vertex* instead, it
  flattened every straight road off its axis — interior feet only.
- **`step` now counts only a jump within one rule**: an edge over the grade
  whose midpoint, asked of its own triangle's rule, is off the mean of its
  ends by more than a quarter of the rise. Continuous steep edges are
  `steep`; both ends on the DEM, `dem_steep`.

Loop box: `step` **18 408 → 144** (worst 11.85 → 3.74 m), `steep` 514,
`dem_steep` 10 070, `welded` 168 558 copies, `contact` 0.00. Every synthetic
specimen with nothing to fix is unchanged from step 1 to the bit; the
Montreux junction box and `ridge` read `step` 0.
`ground::tests::every_drawn_edge_is_welded_or_walled` stays `#[ignore]`d at
**26** on its 150 % flank, where the roundabout ring's axis closes on itself
and the bend blend does not hand over across the seam.

**The cost is 22 340 m² of `split_m2`** — faces inside a surface where the
old code left stretched triangles: 11 k m² in the near pavement between two
roads more than a metre apart (a sidewalk between two terraced streets gets a
wall in its middle), 4.4 k m² where the far pavement stops standing on its
face and drapes, 2.4 k m² in the carriageway, 1.8 k m² in the ballast. A wall
is the model's honest answer to its case functions; whether a pavement
between two terraces should instead be a ramp is the relax's question
(step 3), and `split_m2` is the number that will say what it changes.

**Step 3 is built (2026-09-27), and it is not the relax.** The relax sets a
*decay rate*, so it has no batter slope to keep: a 10 m fill falls away at
whatever the weight makes of it (`relax_vs_cases` max read 49 m on the loop
box). `bench::Ground` takes §3.2's residual `e = z − dem` and gives it a
slope instead: pinned at every outline edge to the room's height less the
natural ground, **clamped to one face** (`MAX_BENCH_FACE_M`) with a wall for
the rest, falling to nothing at 1 in `EARTHWORK_BATTER` perpendicular to its
segment, and blended over `EARTH_BLEND_M` where the nearest segment changes
— a band that narrows to nothing at the outline, so there the field is the
pins' own interpolation and every ground vertex, pinned or not, takes one
function. What that removes, each a discontinuity of the nearest-segment rule
it replaces:

- the step where two outline segments are equidistant and answer differently;
- the all-or-nothing wall: a drop past one face used to refuse the batter
  outright, so the ground stepped between a walled segment and a battered one;
- the lip: a face at an *absolute* 1 in 2.5 never met a hill steeper than that
  and was cut off at 7.5 m standing in it.

**The batter is now relative to the natural ground.** On flat ground nothing
changes; across a 30 % hill the cut face stands at 70 % and meets the ground
2 m out. A batter also does not cross the room: a point on the paving's side
of a segment is not that segment's to answer.

**A footpath runs up the batter now** (2026-09-27). Pavement no road
answers for, or that drapes past one (`Rule::drape`, `Rule::FREE`), is
*passive*: its edges with the ground pin nothing and draw no wall, and its
vertices take the same `natural + residual` the ground does, so a footway
leaving a street runs up the street's batter instead of standing on the raw
terrain beside it — the docs' long-open item. Loop box: `regraded` 18 625
vertices (up to 3.00 m), `wall_m2` 11 972 → 10 886, `kerb_m2` 37 615 →
32 715, `split_m2` 22 333 → 19 550. **What it costs, and why**: `step` 72 →
391, nearly all centimetre-long footpath edges at an outline vertex where
two paved surfaces meet the ground at different heights — a *split* — and
the ground has one copy there, so the earthwork is two-valued at that point
(the ground's own triangles had the same jump since step 3; nothing counted
them). Continuing a paving split into the ground is what removes them, and
is the next item. `off` 3.0 → 3.8 m for the band narrowing at the outline.

**`step` asks the residual, and samples the rule.** An edge whose residual
(`h − natural`) barely changes is the DEM's own steepness (`dem_steep`),
draped or regraded or a street following a flank. Otherwise its rule is read
at 16 points (`STEP_SAMPLES`): a continuous rule moves a sixteenth between
samples however steep or curved (`steep`), a jump puts half the rise between
two. A single midpoint called the DEM's bicubic curvature a jump; with the
samples the 150 % flank roundabout reads 3 steps, not 26.

**The steepest-allowed extension was tried first and is worse.** A cone from
every pin (`e = max(L, min(0, U))`) is continuous, but it couples the pins
*along* the kerb: wherever a road's cut changes faster than the batter's slope
along its own length, one pin's cone overrides the next and each override is
a wall at the kerb — `wall_m2` 43 k against 18 k on the loop box.

Loop box: `wall_m2` **33 091 → 11 979**, the tallest wall 14.8 → 11.8 m (a wall
is now the drop past one face), `off` **21 → 3.0 m**, the drawn ground's
`dem_residual` 0.00/0.89/30.84 → **0.00/0.14/3.00**, `touched` 8.6 → 3.5 %,
`regrade` 143 k → 6.5 k, `contact` 0.00; bench 32 → 18 s, the run 72 → 42 s.
The `relax_vs_cases` diagnostic is retired (it cost ~12 s a run against a
case function that is gone); `relax.rs` stays for the pavement between two
terraces, which is the next place a solve would earn its keep.

**"One height per vertex" (§3.2) and "an edge is split" (§3.3) contradict
each other**, because a kerb vertex has two heights a kerb's rise apart. The
arrangement's `split_vertices` sizes it: **331** on the junction with houses
against that mesh's **84 783** ground vertices, 0.4 %. (Against the
arrangement's own 345 it reads 96 %, which is nearly a tautology — its
vertices *are* the cut points.) So the rule is one vertex per position
**except across a split edge**, and the exception costs a few hundred
vertices.

One trap in `poly::inside`, worth knowing before reusing it: a probe must be
**strictly** interior. Preferring the largest inward step picked a point
lying exactly on the opposite edge of a one-metre band, `contains` said yes,
and every pavement face came back tagged `near` — `walk_far_m2` read 0 and
the bench lifted a band it should have draped. The probe now checks four
neighbours a millimetre out as well.

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
`spanned`, `bridged`/`bridge`, `filled`/`fill`, `shaved`/`shave`,
`notch`/`crest` and `dem_residual`. **The three passes are measured separately on
purpose**: they move the surface for three different reasons, and a composite
number belongs to none of them — read as one it showed a 13.26 m "shave"
against a 4 m budget, which
was the bridging setting a causeway down on its rims. On the loop box:
`blind` 0.16 % / 150 m, `filled` 21.8 % to 14.09 m, `shaved` 9.7 % to 3.96 m,
`notch` 1/43 m, `crest` 12/420 m.

**A mapped span is bridged like a blind run, and for the same reason**
(`reference::spanned_mask`, `spanned`). The blind mask asks the DEM whether it
is standing on the way's deck; the span table says the way has left the ground
here, and a terrain model with its bridges taken out answers with the slot
underneath — the railway in its cutting, the stream in its bed. Read as ground
that slot is the far shoulder of the approach embankment's crest, and the
opening shaves the embankment away as a false bump: at the Montreux rail
overbridge (46.430890, 6.914780) **3.00 m of it** over the last 40 m of
approach, the reference flattened to 398.20 where the DEM climbs to 401.20.
Three things make the rule work, and each cost a measurement:

- **It reaches down the abutment, not only to the annotation's edge.** A span
  boundary is where a mapper clicked; the slot begins where the ground falls
  away, metres earlier. The mask finds the **rim** — the highest ground within
  `ABUTMENT_M` (10 m) of the edge, taken as a rim only where the climb to it
  is steeper than `ABUTMENT_GRADE` (half, which no road in the model is built
  at) — and masks what lies between. The rim itself stays sighted: it is what
  the carry reads from. Walked station by station instead of found, the rule
  ate ten metres of honest approach wherever a deck springs from the *inside*
  of a bowl, and it turned on the grade across the two stations a span
  boundary leaves within millimetres of each other.
- **The passes run twice and are glued at the span boundaries.** Every ground
  station takes the carried pass, so no window reaches into a slot; a station
  *inside* a mapped span keeps the plain one, because there the reference is
  not a target — the profile chords across — but it **is** the evidence
  `partition::derive` reads. Carried across there too, a deck over a 30 m
  gorge reads no departure from the surface it is 30 m above, and an 80 m
  annotation over a 40 m slot stopped being trimmed to it.
- **The source's spans only, never the terrain's own priors.** A promoted span
  is a notch the closing *refused*, and claims level 0 saying so; carried
  across as if a mapper had drawn it, the refusal would be spent twice and
  `NOTCH_FILL_MAX_M` would buy nothing.

On the loop box the conditioning barely moves (`shave` 3.86, `fill` 14.53,
`off` 17.09, all unchanged) because the rule is local to a span's two ends,
and what moves is downstream: bench `contact` 3.78 → 2.89 and `worst`
29.541 → 28.487, `structure mouths` 44/63 → 46/63, profile `bores` 86.71 →
87.55 %, crossing `lift` 14.76 → 14.33.

**It is not what makes a bridged junction look wrong, though.** At the site it
was found on, the road's reference is now the DEM to within 0.25 m and the
solved height at the junction is unchanged: the crossing step's clearance
demand is an *absolute* target (rail + `RAIL_CLEARANCE_M` + the slab, 8.5 m),
so whatever the conditioning gives back the lift takes away. The DEM puts that
road 6.6 m over the rails, the model insists on 8.5, and the 1.9 m difference
is the plinth the junction and its two service roads stand on. That number is
a design standard applied to a structure that already exists, and it tracks
the constant one for one — at 6.0 the plinth is 0.9 m — so it is a decision
about the model, not a defect in it.

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
the reference and `dem_residual` reports the distance from the DEM, which is
the number the departure criterion will threshold.

**That metric is now the run's, not the step's** (`data/plans/one-ground-2026-09-16.md`
§7 step 0). `reference`, `profile`, `crossing`, `partition` and `bench` each
report `dem_residual` as p50/p90/max, **against the same baseline** —
`terrain::height_at`, never the step's own input — so the lines are comparable
down a run and a height is attributable to the step that made it rather than
the step that pays for it. It replaced `reference`'s and `profile`'s `off`,
which were this quantity under a name `bench` uses for a different one (how far
a lattice triangle stands off the *engineered* ground). The numbers are
absolute rather than per-step increments, because a step that undoes its
predecessor's move shows as a smaller number and an increment would hide it.

The population is `Solved::Grade` stations along an axis
(`crossing::residual_of`, one function so every step counts the same thing) and
the drawn ground's own vertices for `bench` (`bench::drawn_residual`).
Deliberately two populations: "how far has the road left the DEM" and "how far
has the ground" are different questions, and **their difference is the
finding**. On the loop box the road reads 0.03/0.40/9.17 and the drawn ground
reads 0.00/0.91/24.63 — an earthwork 2.7× the departure of what it carries,
which is the cost of holding the paved surface level crosswise and is not any
road's solved height. On flat ground the whole departure is `crossing`'s
(`reference` and `profile` read 0.00/0.00/0.00, `crossing` 0.00/1.73/8.50) and
`bench` tops out at 2.99 — `MAX_BENCH_FACE_M` to the centimetre.

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

**Railways are a third family, not a class of carriageway** (`width::Family::Rail`).
The reader keeps the independent classes (`standard_gauge`, `broad_gauge`,
`subway`, `narrow_gauge`, `funicular`) and drops street rail and `unknown`,
which lie on a street or have no formation the model may grant them. A
railway solves a profile, lays a surface (`ballast`, the 3.5 m / 2.6 m track
zone, not the formation) and gets decks and bores exactly as a road does,
and takes no kerb, sidewalk, fillet or room. What is the server's, ported
with its reasons (`data/plans/rail-*.md`): the stiff engineered priors (3 %
on a 2 km vertical curve, metre gauge 7 % on 500 m, the funicular 70 % in a
2.5 m box — the server declared those radii and never read them; here they
are `bend`'s); the **measured ceiling** (`profile::ceiling`: the p90 of the
at-grade bed's grade, capped at `max(30 %, 1.5 × class)`, because a rack
railway is tagged `narrow_gauge` and a funicular runs at 57 %); the
**seniority** in `crossing` (a road always moves, over by `RAIL_CLEARANCE_M`
7 m plus the slab or under a rail deck; a road over a rail bore is not asked
— `senior` — and a road's floor never spreads into a railway through a
connector they share); asphalt over ballast at a level crossing and ballast
over the walk; a ballast bed that ignores buildings (a station roof), and
buildings that yield no passage to a railway. Rail flags reach the reader
because `geoparquet` now reads `rail_flags` beside `road_flags`.

**A railway's structures are a railway's, not a road's.** The deck and the
bore stand `RAIL_SHOULDER_M` (1 m, the server's `STRUCTURE_SHOULDER_M`)
wider than the track zone each side, and the track bed runs to the parapet
— at the bare track zone a metre-gauge viaduct on the flank was 2.6 m wide
and read as a wall. A bore is `structure::tube_m(class)` high inside: 6 m
for standard gauge, for the wire; 5 m for metre gauge and the funicular,
like a road's. One function answers for the structure step, for the
partition's bore test (`partition::bore_cover_m`: tube plus cover, so a
standard-gauge run needs a metre more under the ground before it is a bore)
and for the crossing's clearance over one. The structure line reports
`rail=decks/bores` and `track_m2` beside the road's.

**Overture does not cut a way at every connector**, whatever is said of
junctions above: all 48 of the loop box's rail×road level crossings sit in
the *interior* of both ways. So a level crossing is not found by `crossing`
(it is not two interiors crossing) and not seen by the anchors (it is not a
way end). The profile pins it instead (`contacts`): wherever a railway
shares a connector with another solving way on the ground, **both are pinned
mid-run to the railway's reference** — a level crossing and a switch are at
grade by definition. The limiter pins ends only, so an at-grade run is cut
at every contact and each piece limited between its pins. `level` in the
profile line is the spread at those connectors and reads 0.000. The same
pin mid-run is what `structure abutment` still needs for roads.

On the loop box: 325 railways (139 street-rail features dropped), 87 321 m²
of ballast, `raised` 17 (the rack railway and the funicular among them),
`contacts` 91, `level` 0.000; `crossing` 107 crossings, 45 demands, `senior`
9; 116 decks and 50 bores, of which the railways' are `rail=37/17` over
14 596 m² of track. **What is still open:** `boxed` rose 24/66 →
89/185, because a 2 km rail curve on a mountainside is what the 8 m box
cannot pay for, as it is for a motorway; `clearance` 16/95 short — a road
dipping under a rail deck is held by its own 100 m curve; `cover` −3.49 /
`open` 21, shallow rail galleries whose 5 m tube stands out of the flank
(the server has the same open problem, a per-class bore cover); a rail deck
over a road lowered under it gets no slab where its chord sits at the
*natural* ground, because the structure step reads that rather than the
road's cutting; the funicular's monotone constraint, twin-track welding and
twin bores are not ported; and bench `contact` reads 2.05 m at five
pavement vertices where a pavement touches ballast only at a corner and the
ground reads the ballast's outline there.

The `mesh` step triangulates the carriageway and the pavement **conforming
to the terrain lattice**: each region is ear-clipped and every ear cut by
the lattice's columns, rows and cell diagonals, so every triangle lies in
one terrain triangle and, with its vertices at `height_at`, on the ground
to the ulp — the drape guarantee for areas. The bench step then moves
them off it; on a flat ground the two stay coplanar and z-fight in any
3D viewer, so hide the terrain to see them.
Its summary line: `triangles`/`vertices` and the triangles per material,
`lost_m2` (face area the triangles miss), `off_ground` (the largest height a
triangle's centroid stands off the terrain), `washed`/`lossy` (faces the ear
clipper misread; a washed one was read right from the kernel's union of it),
`slivers`, `welded`, `joined` and `junctions` (the two crack repairs above),
and `crack`, which must read 0. The rings are cleaned together at the
kernel's lattice (`COLLINEAR_M`, 0.1 mm) before clipping: the kernel leaves
straight edges zigzagging by a few hundredths of a millimetre and spikes a
lattice cell wide, and the clipper turns both into T-junctions. The plan view
draws the wireframe in windows under 100 m.

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

**A walk a deck carries is over a span, and does not cut the ground.**
Invariant I3 — only ground-level paving cuts the terrain's hole — was
enforced for the asphalt (`on_ground = paved − spanned`) and for the walk
not at all: `paving.walk` went into the outline whole. So a sidewalk running
along a bridge, which the mapper never tagged as a bridge, opened a hole in
the ground **eight metres under itself**. At the Montreux overbridge the
chain read `room.pavement` true and everything else false, with the pavement
at **404.13** beside a carriageway at 403.65 over a terrain at 395.52 — the
sidewalk is on the deck, exactly where it belongs, and the ground beneath it
had been cut away. `structure::carried` is the same rule one level up and
could not reach it: that asks whether a walk **span** runs along a road's
deck, and this walk has no span to ask about. The walk within
`ROOM_REACH_M` of a span is now folded into the span mask itself, so every
consumer agrees — the hole, the earthwork, and `unmet`, which must not look
for ground under something in the air. `carried` is the area: **1 658 m²**
on the loop box, 67 at the junction. `wall_m2` 41 392 → **40 933** and the
tallest wall 16.2 → 15.7 m; `seam` 4.23 % → 4.73 %, which is the cost — the
carried walk's rim has no ground to meet, by construction.

**`hole_probe` is how that was found, and it is the tool to reach for.** It
walks the bench's own three lines over a grid and prints one character per
metre — `.` ground, `#` asphalt hole, `o` ballast, `w` pavement, `D` deck,
`C` a walk a deck carries — then the full set membership of one named point.
A hole is exactly a point in `outline`, so the map says which term put it
there and the point says it in words.

**Three of its numbers are guards on the walk.** `flown` — walk vertices
whose nearest road stands more than one bench face above the natural ground
under the vertex itself — reads **22 760** on the loop box. `free`, of the
`draped` the ones no road answered for at all, reads **401 327 (88.4 %)**.
`regrade` — pavement left on the raw DEM where the bench's own engineered
ground says something else — reads **157 727 to 4.40 m**. All three are the
docs' open item, "a footpath leaving a street does not run up the batter",
with numbers on it.

**Two of those read 0 for an afternoon, and the cause is worth keeping.**
`Stats::merge` sums the fields the lift reports, and a counter added to
`Stats` but not to `merge` is collected per family and thrown away. `flown`
and `free` were both reported as 0 and both believed, until the geometry at
one point contradicted them. **A new counter is not measured until `merge`
carries it**, and the cheapest check is to read it at a place where the
answer is known by hand.

**A chord answers only where it is.** A vertex over a span is answered by
its sheet's *chords* field and one on the ground by its ground field, and
which it is comes from `Sheets::spanned()` — a mask the polygon kernel
builds, and a kernel's answer has **threads** in it. At the Montreux
overbridge two of them, 0.3 and 0.8 m², lay seven metres from any chord in
the middle of a junction; every vertex they caught went to the chords'
field, which reaches `FIELD_LIMIT_M` (18 m) and clamps to its nearest
station, so it handed back the chord's *end* height. The asphalt stood up
in a **2.4 m vertical fin** along each sliver's edge — 61 near-vertical
carriageway triangles around that one junction. So the chord is believed
only where it is **no further off than the ground the same sheet holds**,
which is a fact about the geometry and one the mask cannot get wrong: on a
deck the chord is underfoot and the approach is a span away, and in a
sliver it is the other way round. Where the mask is right the two are one
profile through one connector and agree anyway, so nothing else moves.
`a_sliver_in_the_span_mask_does_not_lift_the_asphalt` is the check, and it
guards the earthwork too — a sliver was also excusing the `cut` and `fill`
under it as a deck's standoff. On the loop box **`worst` 28.487 → 10.186**
and `step` 0.32 % → 0.31 %, everything else to the unit; near the junction,
3 near-vertical triangles rising at most 0.59 m against 61 and 2.40.

**A junction's legs are blended, and nothing else is.** Each leg is level
crosswise, so on a flank the legs' cross-sections disagree everywhere off
the connector they share — by `g·s` at `s` metres from it where a leg
climbing a flank of grade `g` meets one along its contour — and the
nearest-axis field stepped on the line where two legs are equidistant:
0.875 m on a 15 % specimen flank, with kerb returns whose edges climbed
100–220 %. That is the bumpy junction. So near a connector two or more
axes share (`bench::JOINT_M`, 15 m, fading to nothing at `JOINT_FADE_M`,
25 m) the legs meeting there are blended by how nearly each is the nearest
(`BLEND_M`, 4 m), and the junction is one warped surface: the specimens
read `step` 0 and a steepest edge of 0.33–0.42 against 1.01–2.19 before
(`a_junction_on_a_slope_is_one_surface`), and the loop box reads `step`
0.41 % → 0.35 % for 19.0 s against 17.4. The legs agree at the connector —
the profile pins it — so what is blended is a disagreement that starts at
nothing and grows: a warp, not a ramp between terraces. **Two roads that
meet nowhere near are never blended**, so two terraces on a flank keep the
wall between them (`a_step_no_batter_can_run_is_closed_by_a_wall`).

**The blend width is not settled.** 3 m reads 0.42 on those specimens and
8 m reads 0.31, so the band barely moves them and 4 m is chosen rather
than measured. What is left in the corners is a real twist — a level road
meeting a 15 % side street must warp through its returns — and the worst
edge is always one of those, ~6 m from the connector. `worst` is untouched
by any of it (10.186 m on the box, a cliff the ground walls).

**And the ground answers.** The ground is the one mesh's faces that do not
cut, so **the ground stops at the kerb**: no triangle of it lies under the
asphalt, which is where every artefact of a ground drawn beneath an opaque
surface lives (`data/plans/terrain-hole-plan.md`). Outside the room it is the
natural ground plus the earthwork residual (see "Step 3 is built"): the
room's own height at the outline, a batter falling from it at 1 in
`EARTHWORK_BATTER` of the natural ground, and the natural ground beyond. A
drop past `MAX_BENCH_FACE_M` gets one face of batter and a wall for the rest:
`walled` counts those outline vertices, `wall` the tallest, and **`wall_m2`
of closing face is drawn between the two** — a step nothing spans is a hole
you can see the world through, which is what invariant 9 forbids. **The kerb
gets the same face**: the pavement stands `KERB_RISE_M` over the road it runs
beside, and the edge rule draws that face along the one mesh's own edges,
both rails read by index.

**An abutment is a line across the road, so the mask that ends the hole is
capped square.** The hole is `paving − spanned` (invariant I3: a viaduct
must not punch one in the ground it flies over), and `spanned` used to be
the span's own paving — which is capped **round**, a disc of half the road's
width centred on the connector. Two things followed and both were visible at
the Montreux overbridge: the hole ended in a half-disc, so the retaining
wall wrapped the bridge's nose in a semicircle where an abutment is a
straight face; and the disc reached 2.75 m *back* over the approach, so the
last stretch of embankment stood on terrain nobody had cut and was excused
its own earthwork. `surface::spans_masked` is the same footprint capped
square wherever a span hands over to a ground piece, and `surface::
spans_grouped` — the *paving* — keeps its round cap, because that cap is
what welds the span's ribbon to the approach and a boolean union keeps
touching shapes apart. The two answer different questions and want different
ends. On the loop box: bench `seam` 5.11 % → **4.50 %**, `walled` 1.99 % →
1.87 %, ten thousand fewer ground triangles; nothing else moves.

**And a face of no width is not a face.** The wall is swept along the
outline subdivided at every lattice crossing (`dense`), and some of those
vertices are the same point twice: over 25 m of the Montreux cutting, **9 of
314** wall triangles had no area at all and **92 of 628** plan edges were
under a centimetre. A zero-area triangle has no normal, so a viewer that
computes its own — which the glTF spec requires when a mesh ships none —
shades the wall from a vector that does not exist. Only the exactly
degenerate are dropped; **thinning the rest is a simplification of the
outline and is still to do**, with `wall_m2` as its guard. The sawtooth
still visible along a deep cutting is that outline plus the wall's foot
following the terrain sample by sample.

**And `scripts/preview/index.html` de-indexes before computing normals.**
`gltf::triangles` writes no `NORMAL` and leans on the spec — "when normals
are not specified, client implementations MUST calculate flat normals" —
which is right for a mesh of planar pieces. Three.js `computeVertexNormals()`
on an *indexed* geometry averages across every face sharing a vertex
instead, so a swept wall came out smeared and read as a defect it was not.
**It was not the whole of it**: with flat normals the sawtooth is still
there, which is how we know it is geometry.

**The seam is an index now, and its history is worth one paragraph.** The
ground used to be triangulated apart from the room and take the room's
height at the outline by a lookup keyed at the kernel's grid, with an
eight-cell search for keys a rounding had split; `seam` and `unmet` were that
lookup's misses and `contact` its disagreements. The long-standing
explanation — "a point through one more boolean lands half a grid away" — was
wrong for the booleans, which pin their adapter
(`poly::tests::a_boolean_over_a_snapped_operand_is_idempotent`): the misses
came from **two different regions**, `mesh::by_sheet`'s pieces against the
union the ground was cut from, and no care with the lattice reconciles those.
It was *right* for the buffers and offsets, though, which were not pinned
until 2026-09-27 (`poly::SENTINELS`: a plain ribbon read up to 1.5e-5 m off
the grid, a `dilate` 5e-5 m). Both are gone now: one arrangement, one mesh,
copies by index.

The `structure` step builds what the solved profile implies, and nothing
else: a mapped bridge whose chord never left the ground gets no deck.
**It no longer owns a surface.** A road's or a railway's span is paved by
the `sheet` step, in one polygon with the ground it runs onto, so the
handover at an abutment is a place inside one surface rather than a
boundary between two. **The solid is what is underneath**: over a deck run
a soffit `DECK_THICKNESS_M` (1.5 m) below the roadway with its sides and
end faces, over a bore run a crown `TUNNEL_HEIGHT_M` (5 m) above it with
its walls, open at the portals. What is still paved here is a bore's
floor (a sheet's field would read the road above a hairpin's own tunnel)
and the *walk* — `Field` is built from `Profile`s and a footbridge is not
one, so a walk span has no field to be lifted by and no sheet to join. A **pedestrian span is fitted, not solved**
(most of the box's spans are): a chord between the ground at its own two
ends, no ceiling and no box, and a footbridge's own `WALK_DECK_M` (0.4 m)
deck rather than a road bridge's. **Unless the road already carries it** —
a separated sidewalk mapped as its own bridge is one structure with the
road, so a walk span within the room's reach of a road's span all along is
*carried*: its height is that road's plus the kerb's rise and it builds no
solid.

**There is no abutment block and no pier** — both went in `e8a5b54`. A deck
is a slab of constant thickness full length, so where its roadway runs
closer to the ground than `DECK_THICKNESS_M` the slab passes into the hill,
and `clear` (the least a slab clears the ground) goes negative there: it is
a guard that currently fails, not a number that reads 0.

Its summary line: `spans`, `fitted`, `carried`, `decks`, `bores`,
`galleries`, `rail` (decks/bores), `span_m2`, `bed_m2`, `clear`,
`grounded` (runs that never clear the ground), `cover`/`open` (the crown
against the ground between the portals), `covered`, `mouths`/`walk_mouths`
and `abutment` (a span's end height against the ground piece it lands on).
On the loop box (2026-09-27): 214 spans, 100 fitted, 19 carried, 95 decks,
44 bores, 6 galleries, `rail` 30/18, `clear` −0.99, `grounded` 15,
`covered` 303.0 m, `mouths` 43/58, `abutment` 0.000.

**A portal is where the tube goes into the hill, and the ground in front of
it is a cutting.** Between the line's crossing (where the road goes under
the ground, and where the bore is still elected) and the roof's fit (where
the tube first goes under it) the road runs under the terrain by less than
its tube is tall. The span piece used to carry that stretch, so no surface
step paved it and the bench cut nothing: the terrain lay on the roadway and
the mouth was hill. `structure covered` measures it — tunnel roadway under
the terrain with no tube over it — and it read 24 m on the ridge specimen.
Three rules close it, each where it lives: the partition gives the stretch
back to the ground (`open_portals`, reported as `portal_m`), so it is paved
and benched and walled like any cutting; the tube reaches `PORTAL_M` (2 m)
out over the cutting (`structure::PORTAL_M`), so the edge where the terrain
was cut lies inside it; and across the cap where the cutting meets the
tunnel the bench runs no batter and draws its wall only from the tube's
roof up (`bench::Mouth`) — the headwall, with the opening left open.
`a_portal_is_open` is the check, on a road and a railway.

**A road or railway under the ground whose tube fits nowhere is a
gallery** (`structure::is_gallery`: under the ground somewhere by
`STRUCTURE_MIN_M`, tube under it nowhere). As a bore it drew no tube and
the terrain lay on the road end to end — a road vanishing into the ground
with no entrance. Now the bench cuts its whole footprint (abutment to
abutment, at the structure's width) out of the ground (`bench::Portals`,
which also carries the mouths) and the tube stands over it in the trench.
On the footprint's edge the face closes the ground onto the **tube's
section**, never onto the road: a headwall from the roof up where the hill
is higher, a **footing** from the ground up to the floor where it falls
below — the downhill side of a gallery on a flank, which without it showed
the world through white slivers between the terrain and the tube
(`a_gallery_meets_the_ground_on_both_sides`, read off the wall rule
itself). A bore too short to have a solved run of its own — a stub of a
few metres, deep under the hill — gets its tube over the whole span too,
with the ground left alone.

`structure mouths` counts, for every road and rail tunnel end the source
mapped (not the bbox's cuts), whether a tube's mouth stands there;
`walk_mouths` the same for walks. On the loop box: `covered` 953.1 →
270.9 m, `portal_m` 172, 8 galleries, `mouths` 32/67 → 47/67 and
`walk_mouths` 0/108, `wall_m2` 32 692 → 33 904. **The 20 road and rail
ends still shut have no ground over them**: the way stays within 0.2 m of
the terrain or above it the whole span — short service passages under
buildings the terrain has not got, the funicular and a narrow-gauge line
the model puts 3 m *over* it — and building a tube there would be a
structure built from an annotation (§4.5). **The walks' 108 are the
underpasses**: they pass under a road, a railway or a building rather than
the hill, and a walk does not take part in the crossing step, so nothing
dips it under what it crosses. That is the next step for them.

**A flag builds a structure; a level only orders.** Overture takes a
segment's `level_rules` from OSM's `layer` — what is drawn over what — and
its `is_bridge`/`is_tunnel` flags from `bridge`/`tunnel`. The server's reader
merged the two (the rules, else the flags) and every consumer read a
negative level as a tunnel: Avenue de Naye is mapped at level −1 for 500 m
because it runs under the Viaduc de Chillon, with no tunnel flag, and the
world buried it — a bore the terrain lay on, then a 500 m gallery beside the
lake. The world's reader (`roads::structures`) now builds a structure only
from a flag, taking its ordinal from the level rule of its own sign that
overlaps it; a level no flag covers is a **layer** on the way
(`Way::layers`): ground, with an ordinal the crossing step still reads
(`Way::level_at_arc`) so it knows which of two ways is on top. The
geoparquet `Feature` carries the two signals apart (`rule_runs`,
`flag_runs`); `level_runs` is unchanged and the server's own pipeline with
it. On the loop box the flags and the rules agree on 215 stretches and 23
carry a level alone (18 of them roads below the ground, 691 m); drape reads
`layered` 58 ways, galleries 8 → 7, `mouths` 46/65 and `walk_mouths` 0/86 —
22 of the walks' "underpasses" were a layer too. A structure the flags do
not claim can still be one where the terrain derives it (`partition`),
because there the geometry is the evidence rather than the ordinal.

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
a lawn; both stay ground. **And a hole too narrow to pave is not an island
either** (`room::PAVEMENT_MIN_M`, `kerb::WALK_MIN_M`): `ISLAND_M2` bounded a
hole from above and nothing bounded it from below, so a boolean's leftover
between two ribbons passed every test — small, near a kerb, bordering
asphalt — and was paved. The test is an **erosion** and not an area, because
a scrap is thin rather than small: a three-square-metre refuge is a place
and a forty-metre thread of the same area is not. On the loop box `islands`
**899 → 390** for 161 m² of the 32 472, bench `seam` 4.50 % → 4.25 %.
The plan view marks every kerb
station `kerb_gap` still counts with a red dot in any window under 2 km,
so a gap is found by its marker.

**The step's own constructions are kept apart, and `scraps` is what they
came to.** The pavement leaves this step as one set of regions and five
constructions feed it — the walk the chain handed over, the `band` along a
facade, the `rung` reaching to one, the small holes the union `closed`, and
the `island` in a pocket — and while they were unioned on the way in,
nothing could say which had made any part of the result. They are now
collected separately, and every region too narrow to hold a pavement
([`room::wide_enough`]) is counted against whichever made most of it:
`scraps=907 (293.8 m2) walk:27 band:655 rung:222 island:2` on the loop box,
17 of them at the Montreux junction.

**And a pavement is bigger than the smallest one this step would draw.**
That is a band of `PAVEMENT_MIN_M` over `RUN_MIN_M` — the narrowest strip
over the shortest run of kerb it will start one for, 4.8 m² — and nothing
under it was drawn on purpose. What is under it is what a boolean left: at
the Montreux abutment a 2.84 m² lobe that is the **round cap** of a footway
ribbon, its body cut away by the asphalt and the cap left sitting in the
kerb line, detached from every other piece of pavement. **Unless a wall
explains it** (`WALLED_SHARE`, a quarter): the one pavement drawn smaller
than that is the strip against a facade — a notch, the pocket a kerb return
leaves at a house corner — and `a_notch_in_the_facade_is_pavement_not_asphalt`'s
*whole* pavement is three such regions totalling 1.8 m². A strip squeezed
between a kerb and a wall has the wall along one of its two long sides, so
the specimens read **45 %**; the abutment's lobe reads **0 %**. `loose`
counts what goes: **132 regions, 101.1 m² of 446 030** on the loop box,
9 at the junction. Bench `seam` 4.25 % → 4.23 %, `wall_m2` 41 527 → 41 392,
seven thousand fewer ground triangles — and `wall_gap` 1 → 33 of 69 470,
`kerb_gap` 12 → 38 of 32 557, which is the honest cost: in 32 places the
only pavement beside a kerb *was* one of those orphans.

**Five candidate rules were tried first and every one either broke a
specimen or measured zero.** They are worth knowing because each looks
right:

- a **minimum width** fails `a_notch_in_the_facade_is_pavement_not_asphalt`,
  where the strip between a kerb and a wall two metres off the axis is a
  decimetre wide for twenty metres and is a pavement;
- exempting anything **against a wall** keeps both scraps, which graze one;
- **narrow and short** fails the same notch, a 2 m² isolated patch;
- **provenance alone** fails it too — that specimen's *whole* pavement is
  `band:1 rung:2`, the same constructions the scraps are made of;
- **beside nothing** measured zero, and the boundary probe that had said
  otherwise was sampling the *inside* of each region: outside a
  counter-clockwise ring is to the **right** of each edge.

Two of those five rested on a broken instrument and one on a broken
measurement, and both are worth remembering. The boundary probe was
sampling the **inside** of each region, so everything read "nothing there";
and a probe disc of 25 m around a point *clips* the regions it returns, so
a long region reaching past it came back as a 1.03 m² scrap that does not
exist. **Clip your probe and you will measure your clip.** With a 70 m
radius it is not there, and the one real defect was the 2.84 m² cap.

**The asphalt is complete before the walk is laid, and a span is part of
it.** The three families' paving is mutually disjoint by construction —
`surface` cuts the walk to the carriageway and the ballast, `fillet` to its
own returns, `room` to both — and the `sheet` step then unioned a group's
**span** ribbons into its paving, which none of them had ever been shown. A
sidewalk at an abutment came out inside the road, and 0.12 m over it once
the bench had put the kerb's rise on it: 30 m² on the Montreux junction
model. **Cutting it back there does not work** — by then the two polygons
share a boundary, and a difference along a shared boundary leaves rings the
lattice mesher cannot close (three variants tried, each taking the loop
box's `mesh seam` from 2.7e-9 to **0.74 m**). So the span ribbons are
carried as `Surface::spanned` instead: not paving — `carriageway` stays the
ground pieces' alone, so a viaduct is no part of the street beneath it —
but senior to the walk exactly as the carriageway is. `Surface::senior()`
returns `carriageway ∪ ballast ∪ spanned`, `surface` cuts the walk by it
while both are still raw ribbons that *overlap by an area*, and `kerb`,
`fillet` and `room` read it too, because they are where the pavement
**grows** (`room`'s pockets and islands alone are 60 % of the finished
pavement, and a hole bounded by a deck is not a pocket to pave). The
junction rounding moves with it: `fillet` now closes the corners of a
junction that stands on a structure, per group, with that group's spans
merged in and `SPAN_CORNER_M` of ground asphalt around them — so
`laid_back` re-lays the pavement outside the new kerb and `senior` cuts it,
as for every other return. `sheet` reports **`on_walk`**, the area of the
finished sheets lying on the pavement; it is the invariant of the whole
chain and **zero is the only acceptable reading**. 30 m² → 0.00 on the
junction model and 0.00 on the loop box, with `mesh seam` 2.7e-9 → 2.6e-9,
bench `unmet` 2.59 % → 1.39 % and `seam` 5.51 % → 5.11 %. It costs the walk
998 m² to the spans (`walk_under_asphalt_m2` 11 342 → 12 340) while `room`'s
pavement comes out larger overall, and bench `worst` 10.186 → 15.497 at one
place — the Territet funicular, which climbs its own slope at 60 % and whose
derived deck stands 29.5 m over its own bed.
`--buildings none` runs without buildings; a synthetic network has none
unless `--buildings house:beside?d=2` (a house `d` m off the axis, `notch=`
for a notch in its facade), `house:across?rot=30` (one the way passes
through), `house:row?gap=2` (two along the road) or `house:pair?gap=3` (two
facing across it) says so. The `kerb_gap` and `wall_gap` checks know the
walls: a kerb station with a facade outside it is walled, not bare.

The `building` step stands every footprint up, one building at a time.
The facade reader keeps each building beside the masks, with its height
(measured, else floors × 3 m, else a 5 m guess — the facade line's
`guessed`) and its roof (`roof_shape`, `roof_height`), and **drops what the
source flags `is_underground`** (`underground`): the Veytaux power
station's caverns are mapped on the flank above them, and stood up they
were a 5 m box 105 m in the air on its low side. The construction is the
tiler's `server/src/building_mesh.rs` in metres: a building stands on the
highest ground along its outline and sinks `FOUNDATION_M` (2 m) past the
lowest; `height` is ground to top, so a pitched roof fits under it; a gable
needs a convex quad and a pyramid a convex outline, and a roof the outline
cannot carry is flat (`degraded`). Three things are not the tiler's: the
ground is read where the outline crosses the lattice rather than at its
corners, the walls rise to the roof's rim (the tiler left a skillion open
above its low eave; a gable end is now a wall), and an unmapped rise is half
the short side measured across the longest edge, so a turned house gets the
same roof. `lost_m2` is the roofs' plan area against the footprints'.
`house:beside?d=20&h=10&roof=gabled&rise=3` is the specimen. On the loop
box: 3 223 buildings, 1 219 guessed, 6 underground, 201 gabled, 2 skillion,
170 degraded, `relief` 39.0 m, `lost_m2` 2.8e-7, in 0.01 s.

**`perched` is the rule's cost, and it is 19 % of the loop box**: buildings
whose ground falls away under them by more than their own height, where
standing on the highest ground puts a downhill facade that is more
foundation than building. Montreux is a hillside town and the rule is the
tiler's; a better one needs to know which side a building's height was
measured from. Two more things it does not know: it reads the natural
terrain, not the bench's engineered ground, so a building within a batter's
reach of a road in cutting may show its foundation; and a building a way
runs through is walled to the ground across the passage.

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
