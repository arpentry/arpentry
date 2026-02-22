# Arpentry Client

WebGPU tile viewer written in C, targeting native (macOS/Linux/Windows) and WebAssembly.

## Dependencies

All fetched automatically by CMake:

- [WebGPU-distribution](https://github.com/eliemichel/WebGPU-distribution) — WebGPU C API (wgpu-native by default)
- [GLFW](https://github.com/glfw/glfw) — Window management
- [glfw3webgpu](https://github.com/eliemichel/glfw3webgpu) — GLFW-to-WebGPU surface bridge
- [Brotli](https://github.com/google/brotli) — Compression
- [FlatCC](https://github.com/dvidelabs/flatcc) — FlatBuffers for C
- [Unity](https://github.com/ThrowTheSwitch/Unity) — Test framework

## Building

All commands run from the **repository root**. The root CMakeLists.txt provides shared dependencies (Brotli, FlatCC, Unity) that the client needs.

### Native

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/client/arpentry
ctest --test-dir build --output-on-failure
```

### Emscripten

```bash
# 1. Build flatcc compiler for host
cmake -B build-native -DCMAKE_BUILD_TYPE=Release
cmake --build build-native --target flatcc_cli

# 2. Cross-compile
emcmake cmake -B build-web \
    -DFLATCC_HOST_COMPILER=$(pwd)/build-native/_deps/flatcc-src/bin/flatcc
cmake --build build-web

# 3. Serve (WebGPU requires a secure context — must use localhost, not 127.0.0.1)
python3 -m http.server 8080 --bind localhost -d build-web/client
# Open http://localhost:8080/arpentry.html
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `WEBGPU_BACKEND` | `WGPU` | `WGPU` (wgpu-native) or `DAWN` |
| `FLATCC_HOST_COMPILER` | *(auto)* | Path to host `flatcc` for cross-compilation |
| `CMAKE_BUILD_TYPE` | — | `Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel` |
