# Arpentry Tiler

The tiler generates `.arpa` tile archives from geographic data. Tile generation is framed as a sort problem: features are clipped to tiles, sorted by a space-filling curve key, grouped, encoded, and written to a single archive file.

```
Features → Simplify → Clip to tiles → Sort by tile ID → Group → Build FlatBuffer → Write archive
```

---

## 1. Pipeline

The pipeline reads features, clips each to the tiles it covers, sorts all records by a Hilbert-ordered key, groups records by tile, assembles FlatBuffer tiles, and writes the final `.arpa` archive.

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

### hilbert

Hilbert curve mapping for tile ordering. Converts between (x, y) coordinates and Hilbert curve distance on a 2^order square grid.

```c
uint64_t arpt_hilbert_xy2d(int order, uint32_t x, uint32_t y);
void     arpt_hilbert_d2xy(int order, uint64_t d, uint32_t *x, uint32_t *y);
uint64_t arpt_hilbert_tile_id(int z, int x, int y);
void     arpt_hilbert_tile_id_decode(uint64_t id, int *z, int *x, int *y);
```

### wkb

WKB (Well-Known Binary) geometry parser. Handles types 1–6 (Point, LineString, Polygon, Multi\*), little/big endian, 2D and ISO Z variants. Output is a unified `arpt_geom` struct with SoA coordinate arrays and offset arrays for rings and polygons.

```c
bool arpt_wkb_parse(const uint8_t *data, size_t size, arpt_geom *out);
void arpt_geom_free(arpt_geom *g);
```

### simplify

Douglas-Peucker line simplification. Removes vertices that deviate less than a tolerance from the line between retained endpoints. Operates in-place.

```c
uint32_t arpt_simplify(double *x, double *y, uint32_t count, double tolerance);
```

### clip

Geometry clipping for tile assignment:
- **Points**: bounding box containment
- **Lines**: Liang-Barsky segment clipping
- **Polygons**: Sutherland-Hodgman four-edge clipping

`arpt_assign_tiles()` is the high-level entry: given a geometry and zoom level, it calls back with each (tile, clipped geometry) pair.

```c
void arpt_assign_tiles(const arpt_geom *geom, int zoom, arpt_tile_cb cb, void *ctx);
```

### sort

External merge sort with configurable memory budget. Records are (uint64 key, variable-length data). When the memory budget is exceeded, sorted runs are flushed to temporary files. After `finish()`, a k-way min-heap merge yields a globally sorted stream.

```c
arpt_sorter *arpt_sorter_create(const char *tmp_dir, size_t mem_budget);
bool         arpt_sorter_add(arpt_sorter *s, uint64_t key, const void *data, size_t size);
bool         arpt_sorter_finish(arpt_sorter *s);
bool         arpt_sorter_next(arpt_sorter *s, uint64_t *key, const void **data, size_t *size);
void         arpt_sorter_free(arpt_sorter *s);
```

### tile_build

FlatBuffer tile assembly. Takes features grouped by layer, builds property dictionaries with deduplication, quantizes coordinates to uint16 within tile bounds, and produces Brotli-compressed `.arpt` output.

```c
arpt_tile_builder *arpt_tile_builder_create(arpt_bounds bounds);
bool               arpt_tile_builder_add_feature(arpt_tile_builder *b, const arpt_feature *feat);
void              *arpt_tile_builder_finish(arpt_tile_builder *b, size_t *out_size);
void               arpt_tile_builder_free(arpt_tile_builder *b);
```

### pipeline

Top-level orchestration. Reads features, runs them through simplify → clip → sort → group → tile_build → archive for each zoom level.

```c
bool arpt_pipeline_run(const arpt_pipeline_config *config);
```

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
  --mem <bytes>        Memory budget for external sort (default: 64 MB)
  --threads <n>        Worker threads (default: detected CPU count)
```

Inputs are GeoParquet files keyed by layer index (see Section 9 / `layers`):
`0` terrain, `1` land_cover, `2` bathymetry, `3` water, `4` land,
`5` transportation, `6` land_use. Layer 0 (terrain) is generated, not an input.

Example:

```bash
./build/tiler/arpentry_tiler --output /tmp/test.arpa \
  --bbox 6.0,46.0,7.0,47.0 --min-zoom 0 --max-zoom 14 \
  --input 4:data/naturalearth/land.parquet \
  --input 3:data/naturalearth/lake.parquet
```

---

## 5. Building and Testing

The tiler is native-only (not built for Emscripten).

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build -R "test_hilbert|test_archive|test_wkb|test_simplify|test_clip|test_sort|test_tile_build|test_pipeline" --output-on-failure
```

Tests use the Unity framework. Each module has a corresponding test file in `tiler/tests/`.
