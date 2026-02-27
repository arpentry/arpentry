# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What is Arpentry

Arpentry is a system for stylized 3D globe maps, consisting of three components:

- **Tile format** (.arpt) — A compact binary tile format using FlatBuffers for zero-copy encoding and Brotli for compression. Carries geometry + properties for client-side styling; 3D meshes can include lightweight inline materials. The full format specification is in `FORMAT.md`.
- **Tile server** — A C backend that produces .arpt tiles from source geodata. Includes HTTP serving, procedural terrain generation, and demo tile building (`server/`).
- **Tile viewer** — A WebGPU-based 3D globe renderer written in C, targeting both native (macOS/Linux/Windows via GLFW) and WebAssembly (via Emscripten). Lives in `client/`.

## Build Commands

All commands run from the repository root. CMake 3.20+, C11. All dependencies fetched via FetchContent (no submodules).

```bash
# Native build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Run all tests
ctest --test-dir build --output-on-failure

# Run a single test
./build/common/test_common

# Run the native client (WebGPU viewer)
./scripts/run-native.sh

# Run the WebAssembly client
./scripts/run-web.sh

# Emscripten build (two-step: host compiler first, then cross-compile)
cmake -B build-native -DCMAKE_BUILD_TYPE=Release
cmake --build build-native --target flatcc_cli
emcmake cmake -B build-web -DFLATCC_HOST_COMPILER=$(pwd)/build-native/_deps/flatcc-src/bin/flatcc
cmake --build build-web
python3 -m http.server 8080 --bind localhost -d build-web/client
```

## Project Structure

- `scripts/` — Shell scripts for building and running the native and web targets
- `schemas/` — FlatBuffers schemas (compiled by flatcc at build time)
- `common/` — Shared static library (`arpentry_common`) used by client and server. Includes coordinate/quantization helpers, WGS84 globe math, hashmap, buffer utilities, 3D math, and tile encode/decode. Links FlatCC runtime + Brotli.
- `client/` — WebGPU + GLFW viewer targeting native and WebAssembly
- `server/` — Tile server with HTTP handling, procedural terrain generation, and demo tiles
- `FORMAT.md` — Authoritative format specification for Arpentry Tiles

Generated FlatBuffers headers go to `${CMAKE_BINARY_DIR}/generated/flatcc/`. The `flatcc_generate` custom target compiles schemas.

## Architecture

### Geometry Model

Geometry is a **FlatBuffers union** of four per-topology tables: `PointGeometry`, `LineGeometry`, `PolygonGeometry`, `MeshGeometry`. Each carries only its relevant fields — there is no single table with a topology enum. The union discriminator on `Feature.geometry` identifies the type.

### Coordinate Space

All x/y coordinates are **uint16** with extent 32768 and buffer 16384 per side. Tile proper spans raw values [16384, 49151]. Dequantization: `(qx - 16384) / 32768.0`. Elevation z is **int32 millimeters** above WGS84 ellipsoid (direct value, no dequantization). Coordinates use SoA layout (separate x/y/z arrays).

### MeshGeometry Parts

`MeshGeometry` uses a `Part` struct (16 bytes) grouping index range (`first_index`, `index_count`) with inline material (`Color` RGBA, `roughness`, `metalness`). `color.a == 0` means client-styled.

### Properties

Dictionary-encoded at tile scope: `Tile.keys` (deduplicated strings) and `Tile.values` (typed `Value` union). Each `Feature.properties` entry is a `Property` struct (key index, value index).

## Design Principles

Follow the principles in `DESIGN.md` (deep modules, pull complexity downward, define errors out of existence, etc.).

## Key Conventions

- **FlatCC generates lowercase filenames**: `example_builder.h`, `example_reader.h` (not `Example_builder.h`)
- **FlatCC on newer Clang** needs `-Wno-error=c23-extensions`, `-Wno-error=unused-but-set-variable`, `-Wno-error=implicit-int-conversion` (already configured in root CMakeLists.txt)
- **glfw3webgpu v1.2.0**: the surface function is `glfwGetWGPUSurface()` (not `glfwCreateWindowWGPUSurface`)
- Tests use the **Unity** framework (setUp/tearDown, UNITY_BEGIN/RUN_TEST/UNITY_END pattern)
- Test executables are registered with CTest and live in `common/tests/`, `client/tests/`, and `server/tests/`
