# Arpentry Tiler

The tiler (Rust, in `server/`) generates `.arpa` tile archives from GeoParquet data. Tile generation is framed as a sort problem: features are clipped to tiles, sorted by a space-filling curve key, grouped, encoded, and written to a single archive file.

```
Features → Quadtree walk (simplify + clip per tile) → Sort by tile ID → Group → Build FlatBuffer → Write archive
```

---

## 1. Pipeline

The pipeline runs two phases separated by an external merge sort, both parallel
(`std::thread` + channels, no async runtime):

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
  --terrain <path>     Terrarium DEM PMTiles for per-tile elevation
  --threads <n>        Worker threads (default: detected CPU count)
  --brotli <q>         Brotli quality 0-11 for tile blobs (default: 7)
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

The run ends with a per-stage timing report (read / simplify / clip / sort and
merge / decode / terrain / encode / write) plus row-group pruning and
throughput counters — use it to spot the bottleneck before tuning anything.

---

## 5. Building and Testing

The tiler is native-only and lives in the `server/` crate.

```bash
cd server
cargo build --release
cargo test
cargo test -- --ignored   # real-data + end-to-end tests (need ../data)
```
