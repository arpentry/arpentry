# Design Principles

These shape every module. See `STYLE.md` for code conventions.

## Deep modules

Simple interface, complex implementation. Camera is the canonical example: `arpt_camera_pan_begin` / `arpt_camera_pan_move` hide globe rotation, ray-ellipsoid intersection, and ECEF transforms behind a two-call gesture API. The caller passes screen pixels; the module handles the math.

## Pull complexity downward

When there's a choice between harder for the implementer vs. harder for the caller, take the hit in the implementation. Tile manager handles zoom level computation, visibility culling, async fetching, LRU eviction, and placeholder rendering — the main loop calls `update` and `draw`.

## Define errors out of existence

Design APIs so error conditions can't arise. Make operations idempotent, absorb edge cases internally.

- `arpt_quantize` clamps to [0, 65535] — no out-of-range possible
- `arpt_dvec3_normalize` returns zero vector for zero-length input
- `arpt_camera_zoom_at` clamps altitude to [500m, 40000km]

When errors are unavoidable (decompression, allocation), use `bool` return + output pointer.

## Information hiding

Each module owns its internals. Opaque types (`typedef struct arpt_camera arpt_camera`) prevent callers from depending on struct layout. The camera, renderer, control, and tile manager are all opaque — their headers expose only functions.

## One width per role

Don't offer multiple ways to do the same thing. One compression codec (Brotli). One coordinate extent (32768). One tiling scheme (geographic quadtree). One geometry representation per topology (FlatBuffers union, not enum + shared table). This eliminates configuration, branching, and the bugs that come with them.

## Somewhat general-purpose

Design interfaces that serve the current need while being usable in adjacent situations. `arpt_ray_ellipsoid` serves both screen picking and tile visibility — it wasn't designed for either specifically, just for intersecting rays with the WGS84 ellipsoid.
