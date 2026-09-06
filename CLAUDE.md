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
`sidewalk?d=6`, `corner`, `crossing`) swaps the parquet for a synthetic
network, where a step's output is an assertion rather than a look.
`--until terrain` stops after a step; the steps so far are `terrain`,
`drape`, `ribbon`, `surface`, `kerb`, `fillet`. The plan for the last one
(mesh) is `data/plans/flat-network-2026-09-06.md`.

To look without opening Blender:

```bash
blender -b --python scripts/world-render.py -- /tmp/claude/world.glb \
    /tmp/claude/world.png 0,-8500,5000 0,0,700        # then Read the PNG
```

It prints each layer's vertex and face counts and extents, which must match
the run's summary, then renders from `eye` to `target` in local metres.

For the 2D question — is the outline right — write the plan view instead of
(or as well as) the GLB. It is one SVG group per step, every stroke width a
width in metres, a function of the world alone so `diff` says what moved:

```bash
./world/target/release/arpentry_world --zone data/zones/montreux \
    --bbox 6.89,46.41,6.96,46.45 --terrain flat --spacing 50 \
    --svg /tmp/claude/plan.svg --view -1174,234,-974,434   # metres, x0,y0,x1,y1
rsvg-convert -w 2000 /tmp/claude/plan.svg -o /tmp/claude/plan.png   # then Read
```

`--view` only changes the window (the clip still runs at the bbox), so a
200 m window at 2000 px is 10 px per metre, enough to see a kerb. On a flat
synthetic ground the whole run is under a second.
