# Arpentry Viewer - MVP Specification

A WebGPU-based 3D globe renderer that displays `.arpt` terrain tiles (see `FORMAT.md`). Written in C, compiles natively (macOS/Linux/Windows via GLFW) and to WebAssembly (via Emscripten).

**MVP scope**: terrain mesh only. No vector layers, atmosphere, edge stitching, labels, or offline caching.

---

## 1. Architecture

### Fixed Camera

The camera is fixed at the origin, looking down -Z. Navigation moves and rotates the globe around the camera (not the camera through the scene). The view matrix is identity.

### Coordinate Pipeline

Raw tile coordinates (uint16 + int32) are uploaded directly to the GPU. The vertex shader dequantizes and converts to ECEF. The CPU computes one model matrix per tile in float64.

```
CPU (float64, per tile):
  tile_center_ecef = geodetic_to_ecef(tile_center)
  interest_ecef    = geodetic_to_ecef(interest_point)
  delta            = tile_center_ecef - interest_ecef
  tile_pos_cam     = R_tilt * R_globe * delta + (0, 0, -altitude)
  M_tile           = mat4(R_tilt * R_globe, tile_pos_cam)  →  cast to float32

GPU vertex shader (float32, per vertex):
  1. Dequantize uint16/int32 → geodetic (radians)
  2. Geodetic → ECEF (float32 trig)
  3. Subtract tile center ECEF (same float32 trig → errors cancel)
  4. M_tile * vec4(local_ecef, 1.0)
  5. projection * camera_pos
```

Precision: the tile-relative ECEF subtraction on GPU uses the same float32 trig for both vertex and tile center, so systematic errors cancel. The pipeline preserves the full uint16 quantization precision (9 mm at level 16) at all zoom levels.

### Dequantization (GPU)

```
lon = lon_west + ((f32(qx) - 16384.0) / 32768.0) * (lon_east - lon_west)
lat = lat_south + ((f32(qy) - 16384.0) / 32768.0) * (lat_north - lat_south)
alt = f32(qz) * 0.001
```

### Geodetic → ECEF (GPU)

```
N = a / sqrt(1 - e² * sin²(lat))
X = (N + alt) * cos(lat) * cos(lon)
Y = (N + alt) * cos(lat) * sin(lon)
Z = (N * (1 - e²) + alt) * sin(lat)
```

Computed twice per vertex (vertex + tile center). WGS84 constants (`a = 6,378,137.0`, `e² = 1 - (b/a)²`) are shader constants.

---

## 2. Camera

### Parameters

| Parameter | Description | Range |
|-----------|-------------|-------|
| `longitude` | Interest point longitude | -180° to +180° |
| `latitude` | Interest point latitude | -89° to +89° |
| `altitude` | Height above ellipsoid surface | 500 m to 30,000 km |
| `tilt` | Angle from nadir (0° = straight down) | 0° to 60° |
| `bearing` | Compass direction when tilted (clockwise from north) | 0° to 360° |

Default: nadir view (tilt = 0). Max tilt of 60° guarantees the horizon is never visible (topmost ray reaches 52.5° from nadir with 45° FOV).

### Globe Transform

The camera parameters produce a globe transform (no view matrix):

1. `R_globe`: rotates globe so interest point faces camera (closest to origin along -Z)
2. `R_tilt`: applies tilt and bearing
3. Translation: places interest point at `(0, 0, -altitude)`

These feed into the per-tile model matrices (Section 1).

### Projection

| Parameter | Value |
|-----------|-------|
| FOV | 45° vertical |
| Near | `max(1.0, altitude * 0.01)` |
| Far | `altitude * 10` |
| Aspect | `viewport_width / viewport_height` |

### Input

| Input | Action |
|-------|--------|
| Left-drag | Pan: move interest point (sensitivity scales with altitude) |
| Scroll | Zoom: altitude × 0.95^yoff per scroll tick |
| Right-drag | Tilt (vertical) + bearing (horizontal) |

---

## 3. Tile Management

### Uniform Zoom Level

Limited tilt means all visible surface is at roughly the same camera distance. One zoom level for the entire view:

```
L = floor(log2(root_error * viewport_height / (2 * altitude * tan(fov/2) * 8)))
```

Clamped to `[min_level, max_level]`.

### Visible Tiles

1. Cast rays from screen corners onto the ellipsoid
2. Compute geodetic bounding box (with padding)
3. Enumerate tiles at level L intersecting that box

Flat iteration over a rectangular tile range — no quadtree traversal.

### Tile States

`EMPTY` → `LOADING` → `READY` (GPU buffers uploaded) or `FAILED`.

### Loading

- Max 6 concurrent fetches, prioritized by distance to interest point
- **Fallback**: render deepest available ancestor while target level loads
- **Level transitions**: keep rendering level L until all visible level L+1 tiles are ready
- **Cache**: LRU eviction, ~200 tiles max, retain parent tiles briefly for fallback

### Data Source

1. HTTP server via `arpt_fetch_tile()` (existing). URL: `{base_url}/{level}/{x}/{y}.arpt`
2. Embedded demo tiles (compile-time fallback for offline testing)

Tileset metadata (bounds, min/max level, geometric_error) may be hardcoded for the MVP.

---

## 4. Rendering

### Vertex Data

Raw tile data uploaded to GPU — no CPU vertex processing:

| Buffer | Format | Source |
|--------|--------|--------|
| Position X | `uint16` | `MeshGeometry.x` |
| Position Y | `uint16` | `MeshGeometry.y` |
| Position Z | `int32` | `MeshGeometry.z` |
| Normal | `int8 × 2` | `MeshGeometry.normals` (octahedral) |
| Index | `uint32` | `MeshGeometry.indices` |

Uploaded once per tile, freed on eviction. No re-upload on camera movement.

### Uniforms

| Scope | Uniform | Type |
|-------|---------|------|
| Global | `projection` | `mat4x4<f32>` |
| Global | `sun_direction` | `vec3<f32>` (camera space) |
| Per-tile | `model` | `mat4x4<f32>` (tile-local ECEF → camera space) |
| Per-tile | `tile_bounds` | `vec4<f32>` (lon_west, lat_south, lon_east, lat_north in radians) |
| Per-tile | `center_lon`, `center_lat` | `f32` (radians) |

### Shading

- **Vertex**: dequantize + ECEF + model transform; decode octahedral normals (rotate by model rotation); pass altitude as varying
- **Fragment**: elevation color ramp + diffuse sun lighting with 0.15 ambient
- **Depth**: `Depth24Plus`, clear 1.0, less-equal
- **Clear color**: `(0.05, 0.05, 0.08, 1.0)`

---

## 5. Modules

| Module | Responsibility |
|--------|---------------|
| `globe` | WGS84 math: geodetic ↔ ECEF, surface normals, tile bounds, ray-ellipsoid intersection |
| `camera` | Camera state, globe transform, projection matrix, ray casting |
| `control` | GLFW input callbacks, mouse/keyboard/touch bindings, inertia, fly-to animation |
| `tile_manager` | Zoom level, tile state machine, loading, cache, model matrices |
| `tile_visible` | Visible tile enumeration from camera frustum |
| `tile_decode` | Extract raw FlatBuffer arrays (x/y/z, normals, indices) for GPU upload |
| `tile_fetch` | HTTP tile fetching |
| `renderer` | WebGPU pipeline, GPU buffers, draw calls, uniforms, depth buffer |
| `ui` | WebGPU UI overlay (compass, zoom buttons, tilt controls) |
| `http` | Low-level HTTP client (native sockets / Emscripten fetch) |
| `main` | Window (GLFW/Emscripten), main loop, resize, module wiring |

```
main
 ├── camera → globe
 ├── control → camera
 ├── renderer
 ├── ui
 └── tile_manager → tile_fetch → http, tile_decode, tile_visible, camera, globe
```

### Platform

Platform-specific code behind `#ifdef __EMSCRIPTEN__`: window (GLFW vs canvas), main loop (poll vs `emscripten_request_animation_frame_loop`), tile fetch (sockets vs `emscripten_fetch`), HiDPI (`glfwGetFramebufferSize` vs `emscripten_get_device_pixel_ratio`). Window is resizable with HiDPI support on both platforms.
