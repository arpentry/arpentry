# Arpentry Tiler

The tiler (Rust, in `server/`) generates `.arpa` tile archives from GeoParquet
data. This document owns the **mechanics** — how features become tiles, the
archive layout, the modules, the CLI. It does not own the *model*: what the
heights mean and why they are what they are is `docs/GENERATION.md`, and this
file defers to it wherever the two touch.

Tiling proper is framed as a sort problem: features are clipped to tiles,
sorted by a space-filling curve key, grouped, encoded, and written to a single
archive file. But the sort is only the last of five stages, and it is
deliberately the one with no modelling in it.

```
Assemble ─→ Solve ─→ Ground ─→ ┐
(scene)    (heights) (imprint)  │
                                ├─→ Sort-based tiling ─→ .arpa
Features ─────────────────────→ ┘   (clip, sort, group,
                                     synthesize, encode)
```

---

## 1. Pipeline

### Stages 1–3: the world model, built once

Before a single tile is cut, `pipeline::run` builds the global model that every
tile then reads (docs/GENERATION.md §5):

- **Assemble** (`assemble`) — splice the transportation input's segments into
  corridors, find the junctions where they meet, resolve level annotations into
  corridor-wide structure spans.
- **Solve** (`solve`) — give every corridor a vertical profile, stratum by
  stratum in authority order, with junction continuity and crossing clearance
  holding by construction.
- **Ground** (`ground`) — derive the engineered ground the solved model
  implies: earthworks, bench contact lines, water.

Two derived models follow, both pure functions of the solve:
`synth::carriageway` (the carriageway stretches, handover cuts and intersection
extents the paved surface is built from) and `synth::pavement` (the unioned
road surface itself).

**The pavement bake is bounded by what the run will draw.** Its chunk map is
built from every carriageway source in the model, and the model is everything
the row groups admitted — not what `--bbox` asked for. Unbounded, a two-tile
preview off the Montreux cut unioned all 73 z13 chunks to draw one: 13.6 s of a
17.0 s model stage. `pipeline::pavement_reach` grows the bbox by half a tile per
side (the format buffer, taken at `min_zoom` because the coarsest zoom has the
largest tiles) plus `PAVE_PAD_M`, and chunks outside that are not baked —
73 chunks to 1, and the model stage to 9.2 s (measured in 8dd7fa1, whose zoom
range is not the one the figures below are taken at — compare the ratios, not
the seconds).

**The pruning and the bake's parallelism pull against each other.** `bake` fans
out over chunk keys — `threads.min(keys.len())`, and `bake_chunk` has no inner
parallelism — so a preview pruned to one chunk runs the bake on one core, where
it is the single largest thing in the run. Measured over the cut at the
roundabout on ten threads: at `--min-zoom 16` the reach admits **1 chunk,
803,957 m², 8.0 s** (9.9 µs/m², one thread); at `--min-zoom 13` it admits
**5 chunks, 3,028,777 m², 15.1 s** (5.0 µs/m², five threads). Twice the
throughput per square metre, still nearly twice the wall time — neither dial is
free, and the fastest loop is the smaller one. A bake that split a chunk's work
would be the change that makes the choice go away.

The bounds decide *which chunks are baked, never what one contains*: a chunk is
baked whole, from every source that can influence it, or not at all. That is
what makes a tile drawn in a preview identical to the same tile drawn in the
full archive (invariant 5), and it is asserted rather than assumed —
`bounds_drop_chunks_without_changing_the_ones_kept` compares a bounded bake's
shared chunk ring-for-ring against an unbounded one. Measured end to end, the
pruned and unpruned archives agreed on all 44 metrics of the scorecard then in
place.

All of it is bundled in `pipeline::World` and shared behind `Arc`s. **This is
the load-bearing property of the whole design**: every height an emit worker
writes is a function of the world model and the global terrain lattice, never
of the tile window, so adjacent tiles and successive zooms agree by
construction (GENERATION.md invariant 5) and the tiling below carries no
modelling responsibility at all.

### Stages 4–5: the tiling

Two phases separated by an external merge sort, both parallel (`std::thread` +
channels, no async runtime):

**Phase 1 — process.** Each input's Parquet row groups become work items
(pruned against the tiling bbox using the `bbox` column's row-group
statistics, so out-of-bounds row groups are never read). Worker threads stream
features from their row groups — only the geometry column and requested
attribute roots are decoded — and fan each feature into per-tile sort records
via a quadtree walk: the feature is clipped to its first emitted zoom's
tile(s), then recursively clipped into child tiles down to `max_zoom`. Each
node emits one record, re-simplified for that zoom from the carried geometry
(held at `tolerance(max_zoom)` detail). Work is proportional to the records
emitted, not `covered tiles × vertices`. Features wholly smaller than one
screen pixel at a zoom are skipped at that zoom. Every worker feeds its own
external sorter.

**Phase 2 — emit.** The per-worker sorters merge k-way into one
Hilbert-ordered stream. A dispatcher thread groups consecutive records by tile
id into jobs; a worker pool decodes each job, builds the terrain mesh (DEM or
flat), assembles the FlatBuffer, and Brotli-compresses it; the writer thread
restores stream order with a small sequence-keyed heap and appends tiles to
the archive. Workers own their DEM readers — the Hilbert order keeps their
tile caches hot.

Geometry synthesis (stage 4, `synth`) happens inside the emit job: each
feature carries a `Synth` tag decided back in phase 1, and the generator for
that tag reads the shared world model. A generator degrades rather than fails
(invariant 6) — a structure whose corridor has no profile falls back to a
draped road, something plain rather than something wrong.

### Sort key

Every clipped feature becomes a sort record with a 64-bit key:

```
bits [63..16]  tile_id  (48 bits)  — Hilbert-ordered, zoom-prefixed
bits [15..12]  layer    (4 bits)   — up to 16 layers
bits [11..0]   rank     (12 bits)  — feature priority within layer
```

This layout ensures that all features for the same tile are adjacent after sorting, ordered by layer then rank.

### Tile ID

The tile ID encodes zoom level and spatial position using a Hilbert curve:

```
bits [47..42]  zoom     (6 bits, max 63)
bits [41..0]   hilbert  (42 bits; the curve uses 2·z bits, so z ≤ 21)
```

At zoom z, the tile grid is 2^z columns × 2^z rows (one root tile at z0). The Hilbert curve is indexed over this 2^z square (order = z). Tiles are grouped by zoom, then ordered spatially within each zoom for cache-friendly access.

---

## 2. Archive Format (.arpa)

The `.arpa` format is a single-file tile archive. All multi-byte integers are little-endian.

```
┌──────────────────────────────────────────────┐
│ Header (128 bytes)                           │
│   magic "arpa", version, zoom range, bounds, │
│   root_error, tile_count, dir/meta offsets   │
├──────────────────────────────────────────────┤
│ Tile Data                                    │
│   Sequential Brotli-compressed .arpt blobs   │
├──────────────────────────────────────────────┤
│ Directory                                    │
│   Sorted array of entries (40 bytes each),   │
│   binary-searchable by Hilbert tile ID       │
├──────────────────────────────────────────────┤
│ Metadata                                     │
│   Brotli-compressed .arpi blob               │
└──────────────────────────────────────────────┘
```

### Header (128 bytes)

| Field | Type | Description |
|-------|------|-------------|
| magic | uint32 | `0x61727061` ("arpa") |
| version | uint32 | Format version (currently 1) |
| min_zoom | uint8 | Minimum zoom level |
| max_zoom | uint8 | Maximum zoom level |
| bounds | 4×float64 | West, south, east, north (WGS84 degrees) |
| root_error | float64 | Root geometric error |
| tile_count | uint64 | Number of tiles in the archive |
| dir_offset | uint64 | Byte offset to directory |
| meta_offset | uint64 | Byte offset to metadata |
| meta_size | uint64 | Size of metadata blob |

### Directory entry (40 bytes)

| Field | Type | Description |
|-------|------|-------------|
| hilbert_id | uint64 | Hilbert-ordered tile ID (search key) |
| offset | uint64 | Byte offset to tile data |
| size | uint64 | Compressed tile size |
| z | uint8 | Zoom level |
| x | uint32 | Tile column |
| y | uint32 | Tile row |

The directory is sorted by `hilbert_id` for binary search lookup.

### Writer

The writer appends tile data sequentially, accumulates directory entries in memory, then on `finish()` sorts the directory by Hilbert ID and writes it followed by the header.

### Reader

The reader mmap's the file, reads the header, and serves tile lookups via binary search on the directory. Tile data pointers are valid until the reader is closed.

---

## 3. Modules

All in the `server/` crate (`src/`).

### geoparquet

Streaming GeoParquet reader. Opens footers only; `features(row_groups, attrs)`
yields features batch by batch with projection pushdown (geometry + requested
attribute roots), and `row_groups_intersecting(bounds)` prunes row groups via
the `bbox` struct column's statistics. Dotted attribute paths
(`cartography.min_zoom`) descend into nested structs; absent columns are
skipped, so one column list serves Overture and Natural Earth.

### wkb

Hand-rolled WKB parser/writer (types 1–7, little/big endian, ISO-Z/EWKB; Z/M
discarded) producing `geo-types` geometries.

### simplify

Douglas–Peucker simplification over `geo-types` (iterative, stack-safe), plus
`area`/`length` measures used for sub-pixel dropping.

### clip

Rectangle clipping for tile assignment: points by containment, lines by
Liang–Barsky, polygons by Sutherland–Hodgman. `assign_tiles` clips a geometry
to every tile it covers at one zoom; `candidate_range` exposes the buffered
candidate-tile range the pipeline's quadtree walk starts from.

### sort

`ExternalSorter`: external merge sort with a memory budget — records
accumulate in memory, spill as sorted runs, and `into_sorted()` k-way merges
them. `sort::merge(sorters)` joins many independently filled sorters (one per
phase-1 worker) into a single globally sorted stream.

### record

Wire codec for sort-record payloads (id, WKB geometry, properties).
`RecordEncoder` serializes a feature's id + properties once and stamps out
per-tile records, since a feature can fan out to thousands of tiles.

### tile_build

FlatBuffer tile assembly: property dictionaries with deduplication, uint16
quantization within tile bounds, the geometry union, and Brotli compression
(`DEFAULT_QUALITY` 7 — measured ~30× faster than quality 11 with equal size).

### terrain / dem / pmtiles

Terrain meshes for every tile: `flat_mesh` when no DEM is configured,
`elevated_mesh` sampling a Terrarium PMTiles DEM (Mapterhorn) with per-vertex
elevation and cross-tile-continuous normals.

`dem` is one facade over two grounds. A `--terrain` value that names a kind
word — `flat`, `ramp`, `hill`, `step` — builds an analytic `dem::Field` instead
of opening an archive, and every one of the fifteen places that open a DEM gets
it without learning which it holds. That is the isolation harness's terrain
dial: hold the features fixed and move only the ground, and a defect present on
a plane is in the construction while one that first appears on a hill is in the
ground or in how the ground is read. A field also has no gaps, no zoom and no
cache, so it answers the same height everywhere at every zoom — one fewer thing
that can move under a measurement. `scripts/terrain-rungs.sh` runs one site
across the whole dial.

### pipeline

Top-level orchestration (`pipeline::run(&Config)`): the two parallel phases
described in §1, per-stage timing/counter stats, and atomic archive output
(temp file + rename).

---

## 4. CLI

```
arpentry_tiler [options]

  --output <path>      Output .arpa archive path (required)
  --input <N:path>     GeoParquet input keyed by layer index N (repeatable)
  --bbox <w,s,e,n>     Geographic bounds in degrees (default: world)
  --min-zoom <z>       Minimum zoom level (default: 0)
  --max-zoom <z>       Maximum zoom level (default: 4)
  --tmp <dir>          Temp directory for sort runs (default: system temp)
  --mem <bytes>        Memory budget for external sort (default: 64 MiB)
  --terrain <path|spec>
                       Terrarium DEM PMTiles for per-tile elevation, or an
                       analytic ground (the terrain dial):
                         flat[?h=400]
                         ramp?grade=0.03[&bearing=90][&h=][&at=lon,lat]
                         hill?amp=60&radius=400[&h=][&at=]
                         step?rise=3[&width=0][&bearing=90][&h=][&at=]
                       `bearing` is compass degrees (0=N, 90=E) and names the
                       direction the ground rises in; `at` is the origin of the
                       local metric frame and defaults to the centre of --bbox,
                       which the CLI writes back into the spec so the value the
                       run records is self-contained
  --threads <n>        Worker threads (default: detected CPU count)
  --brotli <q>         Brotli quality 0-11 for tile blobs (default: 7)
  --no-breaklines      Plain lattice terrain: no bench contact lines, and no
                       hole (there is no constrained mesh to cut)
  --no-hole            Draw ground under the asphalt again, so an A/B re-tile
                       of the hole is a flag rather than a patch
  --plan-lines         Also draw the plan-space network the surfaces were
                       built from: the network view (below)
```

Inputs are GeoParquet files keyed by layer index (see `layers`):
`0` terrain, `1` land_cover, `2` bathymetry, `3` water, `4` land,
`5` transportation, `6` land_use, `7` building, `8` poi, `9` boundary.
Layer 0 (terrain) is generated, not an input. Building footprints and POI
points get a per-feature base elevation sampled from the DEM (when one is
configured), written as a constant `z` array, so buildings and labels sit on
the terrain.

Example:

```bash
./server/target/release/arpentry_tiler --output /tmp/test.arpa \
  --bbox 6.0,46.0,7.0,47.0 --min-zoom 0 --max-zoom 14 \
  --input 4:data/naturalearth/land.parquet \
  --input 3:data/naturalearth/lake.parquet
```

### The network view

At the surface zooms a tile carries the *result* of the network synthesis and
nothing else: one unioned mesh per surface family, with the cartographic stroke
of every way that fed it deleted, because a line painted over its own asphalt is
a second coat of the same paint. That is right for a map and useless for anyone
asking why the asphalt is *there*. The two things that question needs — the
centreline the synthesis consumed and the edge the model says it paved — are
exactly the two the tile does not carry, so a disagreement between plan and
surface is invisible in the only place it could be seen.

`--plan-lines` puts both back, as their own classes in the transportation
layer, over the surface they produced (`server/src/synth/plan.rs`):

| Class | What it is |
|-------|-----------|
| `plan_axis_*` | The source segment's centreline, at its solved surface height — where the surface is, according to the model |
| `plan_edge_*` | The same segment offset to `sect_a`/`sect_b`, one line per side — where the model says its own surface *ends* |

`*` is the surface family: `road`, `rail`, `walk`, `path`. The union buffers
and dissolves these polylines, so a gap between `plan_edge_*` and the drawn rim
beside it is the union's doing and nothing else's. It draws the `SourceSeg`
population — the one input the drawn surface actually has — rather than the
Overture geometry: a sidewalk has no source line at all (it is derived from the
street it rides), and a corridor's line is several joined segments. What a
reader compares against the mesh is what the mesher was given.

Pair it with the client's `--hide-terrain` and `style-network.json`; the ground
is the thing every other surface is drawn *against*, so hiding it is what makes
a band left hanging in the air over the hole cut for it legible as such
(`docs/VIEWER.md` "The network view").

```bash
arpentry_tiler --output /tmp/net.arpa --plan-lines ...        # tile it
arpentry_server /tmp/net.arpa style-network.json --bundle     # style it
arpentry_client --archive /tmp/net.arpa --hide-terrain \
    --lon 6.929 --lat 46.420 --alt 560 --tilt 50 \
    --headless --screenshot /tmp/net.png
```

**An archive carrying the plan lines is a debugging archive, not a map**, and
the run summary says so on its `plan` line. It is not safe to *serve*: the
transportation layer carries three features per source segment that no map
style draws. Any future check must drop every `plan_*` class on the way in, or
it will measure the machinery instead of the map — the emission poisoned a
surface metric exactly that way once.

### The run summary

The run ends with a per-stage timing report (read / simplify / clip / sort and
merge / decode / terrain / encode / write) plus row-group pruning and
throughput counters — use it to spot the bottleneck before tuning anything.

**`total` is the whole run, and its `of which` line says what the stages
account for.** It has to be measured rather than summed: the model stage runs
before phase 1, and the probes run between the
two, so anything added there is inside a wall clock and outside every sum.
`total` was `phase1 + phase2` until it was checked against one — it named
10.8 s of a 25.5 s run at `--max-zoom 16`, and 5.8 s of 27.2 s once
`--min-zoom 13` moved the work into the model stage, with `features/s` and
`tiles/s` overstated by the same factor. The remainder is printed on every
run, zero included, for the reason the `cdt` line below is.

Two of its lines report things no other tool can see:

```
consistency  junction step max ..., clearance shortfall max ..., N demands dropped
cdt          N tiles lost their breaklines, M fell back from the one mesh
```

`cdt` counts the tiles whose triangulation the geometric kernel refused.
spade's constraint splitting asserts rather than returning on some
nearly-degenerate configurations, so `terrain_cdt` catches the panic and takes
the fallback each caller already promises: the plain lattice for
`constrained_mesh`, the pre-S5 separate meshes for `one_mesh_full`. Both are
the right answer for the tile that hit them (invariant 6: plain, not wrong)
and both are invisible downstream — an archive reader cannot distinguish a
refusal from a tile that never had breaklines, because a plain lattice is a
plain lattice either way. A nonzero count means those
tiles were built by a different construction than their neighbours, which is
worth knowing before blaming a seam near one on a code change.

The line is printed on every run, zero included: a fallback is silent
everywhere else, so a line that appeared only on failure would make its own
absence unreadable.

`consistency` is scene-wide, not zone-wide. With a cut input (below) the
boundary contributes to it even when every measured tile is fine, so treat it
as a smell rather than a metric.

### Cutting the inputs to a zone

Row-group pruning bounds what the tiler *reads*, not what the world model
*solves*. One row group of a 954 MB `segment.parquet` is a large piece of the
canton, so a small `--bbox` still assembles, solves and grounds every corridor
in the groups it touched: measured at 23,611 corridors and 51.6 s of a 74 s
run for a 25-tile bbox, and flat in the size of the bbox.

`scripts/cut-zone.sh <name> <w,s,e,n> [--margin <deg>]` cuts every layer and
the DEM to one zone plus a margin, once, and `run-overture-ch.sh --data <dir>`
tiles from the cut. The same run then takes 45 s with the model stage at
25.5 s. The script's header carries the margin calibration and the reason a
cut zone's numbers are not comparable to any taken over full inputs: the full
run admits corridors that reach past the DEM extract and solve against the
flat-0 fallback.

### Where the model stage actually goes

The `of which` line under `model` splits it into assemble, solve, the pavement
bake and the remainder (seniors, walk bands, crossings, the walk fit, the walk
graph — itself split by the `other is` line). It exists because "the model
stage is slow" named no stage, and the obvious inference from the paragraph
above — that a small bbox pays for assembling the whole row group — turns out
to be the wrong lever. Over the Montreux cut at the roundabout, two tiles at
z16: **assemble 0.2 s, solve 3.1 s, pavement 8.0 s, other 2.7 s** of a 14.1 s
model stage. Assembling 8506 corridors is not what the run spends its time on;
paving them is.

The split is a remainder, not a sum: `other` is `model − assemble − solve −
pavement`, so a stage nobody has named yet still shows up in it. **The
pavement bake is subtracted because it runs inside the model stage, not beside
it** — reading its own line as an addition is how a 13.6 s bake once spent a
session looking like a 2.1 s remainder.

`--stage-out <path>` writes the assembled scene to a snapshot and `--stage-in
<path>` reuses it, guarded by what it was built from — the bbox, the ground
and every input's size and mtime refuse out loud, and a rebuilt tiler warns,
since it cannot tell which stage the rebuild touched. A reused scene
reproduces every archive metric exactly. It also saves 0.2 s of 14.1 s, which
is the above number and not a reason to use it yet. `Snapshot` holds a `scene`
and nothing else; the solved model (3.1 s) and the ground are the ones that
would pay, and each needs a table in `schemas/stage.fbs` plus its half of the
codec — the format's room for them is FlatBuffers' additive fields, not
declared structure.

---

## 5. Building and Testing

The tiler is native-only and lives in the `server/` crate.

```bash
cd server
cargo build --release
cargo test
cargo test -- --ignored   # real-data + end-to-end tests (need ../data)
```
