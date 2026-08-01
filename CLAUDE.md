# CLAUDE.md

Guidance for Claude Code when working in this repository. Before asking the human for help, use the build, test, and verification commands below to diagnose and fix issues yourself.

## Documentation

Read these docs before making changes to understand the design and conventions:

| Document | What it owns |
|----------|-------------|
| `docs/MOTIVATION.md` | Project motivation and background |
| `docs/DESIGN.md` | Design principles (deep modules, pull complexity downward, define errors out of existence) |
| `docs/STYLE.md` | C coding style guide |
| `docs/FORMAT.md` | Tile format specification: geometry model, coordinate space, properties, FlatBuffers schema |
| `docs/VIEWER.md` | Viewer specification: coordinate pipeline, tile management, rendering |
| `docs/CONTROL.md` | Map control specification: camera parameters, input bindings, pan/zoom/rotate, inertia, fly-to |
| `docs/VERIFICATION.md` | Measuring an emitted archive against the invariants: the scorecard, its thresholds, the scenario corpus |

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

## Verifying Generated Geometry

Before reaching for a screenshot, measure. `arpentry_verify` scores an emitted
archive against the `docs/GENERATION.md` §5 invariants — asphalt buried by the
drawn ground, level ordering inverted, tile-seam steps, manufactured retaining
walls, structures drifting between zooms — in about 7 s over a city extract:

```bash
cd server && cargo build --release
./target/release/arpentry_verify ../data/overture-ch/preview.arpa

# Judge a change by the diff, not by an impression. Exits 1 on any regression.
./target/release/arpentry_verify ../data/overture-ch/preview.arpa \
    --baseline verify/baseline-montreux-z16.json

# Scope to one place, or to one canonical situation from GENERATION.md §4.
./target/release/arpentry_verify ../data/overture-ch/preview.arpa --at 6.9290,46.4200
./target/release/arpentry_verify ../data/overture-ch/preview.arpa --scenario S5
```

When a metric names a place, cut a section there rather than flying a camera to
it. A 3/4 perspective view is close to the worst possible image for judging
heights; in section a 3 m step is a 3 m step:

```bash
./target/release/arpentry_verify ../data/overture-ch/preview.arpa \
    --at 6.928167,46.426206 --bearing 90 --length 120 --section /tmp/claude/cut.svg
rsvg-convert -w 1100 /tmp/claude/cut.svg -o /tmp/claude/cut.png   # then Read the PNG
```

Read `docs/VERIFICATION.md` before adding or interpreting a metric — in
particular the section on where the thresholds come from, since every
structure-versus-surface check has a legitimate contact band that a naive zero
threshold reports as a defect.

**Use a screenshot to discover a defect, then write the check before fixing
it.** The render finds a class of defect once; the check keeps it dead. A defect
that stays screenshot-only is one nobody can tell has come back.

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
--screenshot <path>   Capture a PNG after tiles load, then exit
```

With `--screenshot`, the client waits for all visible tiles to load, captures one frame, prints `[SCREENSHOT] saved <path>`, and exits with code 0 on success.
