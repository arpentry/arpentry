# Arpentry

A system for stylized 3D globe maps, written in C.

Three components:

- **Tile format** (`.arpt`) — Compact binary tiles using FlatBuffers (zero-copy) and Brotli (compression). Carries geometry and properties for client-side styling; meshes can embed lightweight materials.
- **Tile server** — Produces `.arpt` tiles from source geodata. HTTP serving, procedural terrain, demo tiles.
- **Tile viewer** — WebGPU 3D globe renderer. Native (macOS/Linux/Windows via GLFW) and WebAssembly (via Emscripten).

## Building

Requires CMake 3.20+ and a C11 compiler. All dependencies fetched via CMake FetchContent.

```bash
# Native build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Run tests
ctest --test-dir build --output-on-failure

# Run the native viewer
./scripts/run-native.sh

# Run the WebAssembly viewer
./scripts/run-web.sh
```

Emscripten cross-compilation (two-step: host compiler first, then cross-compile):

```bash
cmake -B build-native -DCMAKE_BUILD_TYPE=Release
cmake --build build-native --target flatcc_cli
emcmake cmake -B build-web -DFLATCC_HOST_COMPILER=$(pwd)/build-native/_deps/flatcc-src/bin/flatcc
cmake --build build-web
python3 -m http.server 8080 --bind localhost -d build-web/client
```

## Project Structure

| Directory | Description |
|-----------|-------------|
| `common/` | Shared library: coordinate helpers, WGS84 math, tile encode/decode, hashmap, buffer utilities, 3D math |
| `client/` | WebGPU + GLFW viewer |
| `server/` | Tile server: HTTP handling, procedural terrain |
| `schemas/` | FlatBuffers schemas (compiled at build time) |
| `scripts/` | Build and run scripts |

## Documentation

| Document | Description |
|----------|-------------|
| `docs/FORMAT.md` | Tile format specification: tiling scheme, geometry encoding, properties, coordinate system |
| `docs/VIEWER.md` | Viewer specification: coordinate pipeline, tile management, rendering |
| `docs/CONTROL.md` | Map control specification: pan, zoom, rotate, inertia, fly-to, input bindings |
| `docs/DESIGN.md` | Design principles: deep modules, information hiding, error handling |
| `docs/STYLE.md` | C coding style guide |

## AI Assistant

`CLAUDE.md` provides context for [Claude Code](https://claude.ai/code): conventions, gotchas, and pointers to the documentation above.

## Dependencies

- [FlatCC](https://github.com/dvidelabs/flatcc) — FlatBuffers compiler and runtime for C
- [Brotli](https://github.com/google/brotli) — compression
- [WebGPU-distribution](https://github.com/nicebyte/webgpu-distribution) — WebGPU headers and native backend
- [GLFW](https://www.glfw.org/) — windowing (native target)
- [glfw3webgpu](https://github.com/nicebyte/glfw3webgpu) — GLFW/WebGPU bridge
- [Unity](https://github.com/ThrowTheSwitch/Unity) — test framework

## Acknowledgments

The server's networking layer (HTTP, TLS, buffer management) derives from [pogocache](https://github.com/tidwall/pogocache) by Josh Baker, MIT license.
