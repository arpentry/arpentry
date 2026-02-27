# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Documentation

See `README.md` for project overview, build commands, and project structure.

| Document | What it owns |
|----------|-------------|
| `docs/DESIGN.md` | Design principles (deep modules, pull complexity downward, define errors out of existence) |
| `docs/STYLE.md` | C coding style guide |
| `docs/FORMAT.md` | Tile format specification: geometry model, coordinate space, properties, FlatBuffers schema |
| `docs/VIEWER.md` | Viewer specification: coordinate pipeline, tile management, rendering |
| `docs/CONTROL.md` | Map control specification: camera parameters, input bindings, pan/zoom/rotate, inertia, fly-to |

Follow `docs/DESIGN.md` principles and `docs/STYLE.md` conventions in all code.

## Key Conventions

These are gotchas not documented elsewhere:

- **FlatCC generates lowercase filenames**: `example_builder.h`, `example_reader.h` (not `Example_builder.h`)
- **FlatCC on newer Clang** needs `-Wno-error=c23-extensions`, `-Wno-error=unused-but-set-variable`, `-Wno-error=implicit-int-conversion` (already configured in root CMakeLists.txt)
- **Generated FlatBuffers headers** go to `${CMAKE_BINARY_DIR}/generated/flatcc/`. The `flatcc_generate` custom target compiles schemas.
- **glfw3webgpu v1.2.0**: the surface function is `glfwGetWGPUSurface()` (not `glfwCreateWindowWGPUSurface`)
- **Tests use Unity**: setUp/tearDown, UNITY_BEGIN/RUN_TEST/UNITY_END pattern. Test executables live in `common/tests/`, `client/tests/`, and `server/tests/`, registered with CTest.
- **Run a single test**: `./build/common/test_common` (or any test executable directly)
