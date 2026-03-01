# Arpentry Tiles (.arpt) - Format Specification

## Context

This document defines **Arpentry Tiles** (`.arpt`), a binary tile format for stylized 3D globe maps. The format carries geometry and properties for client-side styling, while 3D meshes can include lightweight materials for visual identity. It uses FlatBuffers for zero-copy binary encoding and targets a C implementation.

---

## 1. Design Principles

### What it IS

- A compact binary tile format for stylized 3D globe maps
- Geometry + metadata carrier; most styling deferred to client, models can carry lightweight materials
- Globe-native: designed for Earth curvature, ECEF precision, tile-local coordinates
- Zero-copy via FlatBuffers: read directly from buffer without deserialization

### What it is NOT

- Not a photorealistic format (no PBR materials, textures, animations)
- Not a general 3D interchange format
- Not a flat/2D map format (globe rendering is primary)
- Not a CAD/BIM format (buildings are extruded footprints, not architectural models)

### Principles

1. **Compact**: 50-500 KB per tile (terrain + vectors)
2. **Stylable**: Vector features carry geometry + properties (client-styled); models can carry lightweight materials (color) for visual identity
3. **Globe-native**: Tile-local quantized coordinates avoid float32 precision issues
4. **Zero-copy**: FlatBuffers hot path (feature iteration) has no allocations
5. **Simple**: One width per role (no conditional sizing). One compression codec. Derivable values are not stored. Each concept lives in one place.
6. **Evolvable**: FlatBuffers schema evolution for backward compatibility

---

## 2. Tiling Scheme

### Geographic Quadtree on WGS84

- 2 root tiles at level 0: west hemisphere (-180..0 lon) and east hemisphere (0..180 lon), each spanning -90..+90 lat
- Each tile subdivides into 4 children (standard quadtree)
- Tile addressing: `{level}/{x}/{y}.arpt`
  - x: 0 to 2^(level+1) - 1 (columns)
  - y: 0 to 2^level - 1 (rows, south to north)
- Max 22 levels supported (sub-millimeter precision)

### Tile Bounds

```
lon_west  = -180 + x * (360 / 2^(level+1))
lon_east  = lon_west + (360 / 2^(level+1))
lat_south = -90 + y * (180 / 2^level)
lat_north = lat_south + (180 / 2^level)
```

### Level of Detail

- Each tile has a geometric error (meters) describing its approximation error
- Client computes screen-space error: `sse = (geometric_error * viewport_height) / (2 * distance * tan(fov/2))`
- Refine (load children) when `sse > 8` pixels
- REPLACE refinement only: parent hidden when children loaded
- Deterministic error: `geometric_error(level) = root_error / 2^level`

### Tileset Metadata

A flat JSON file (no recursive children tree) describes the tileset:

```json
{
  "arpentry_tiles": "1.0",
  "name": "Example Tileset",
  "bounds": { "west": -180.0, "south": -90.0, "east": 180.0, "north": 90.0 },
  "elevation_range": { "min": -500.0, "max": 4800.0 },
  "min_level": 0,
  "max_level": 16,
  "geometric_error": 50000.0,
  "tile_url_template": "{base_url}/{level}/{x}/{y}.arpt",
  "layers": [
    { "name": "terrain",  "geometry_types": ["Mesh"],    "min_level": 0,  "max_level": 16 },
    { "name": "surface",  "geometry_types": ["Polygon"], "min_level": 0,  "max_level": 16 },
    { "name": "highway",  "geometry_types": ["Line"],    "min_level": 8,  "max_level": 16 },
    { "name": "tree",     "geometry_types": ["Point"],   "min_level": 14, "max_level": 16 },
    { "name": "building", "geometry_types": ["Polygon"], "min_level": 13, "max_level": 16 },
    { "name": "poi",      "geometry_types": ["Point"],   "min_level": 10, "max_level": 16 }
  ]
}
```

The `geometry_types` array uses these string values, mapping to the Geometry union members in the schema:

| JSON string | Schema type |
|-------------|-------------|
| `"Point"` | `PointGeometry` |
| `"Line"` | `LineGeometry` |
| `"Polygon"` | `PolygonGeometry` |
| `"Mesh"` | `MeshGeometry` |

---

## 3. Tile Content Structure

### 3.1 Layers

Each tile contains named layers. A layer has:

- **name**: string identifier
- **features**: vector of features

The tile contains shared property dictionaries (`keys`, `values`) used by all layers (see Section 4).

Two spec constants govern the coordinate space (not stored in the tile):

- **Extent**: 32768 — the tile spans this many units along each axis
- **Buffer**: 16384 — units of overflow per side for geometry beyond tile edges

The uint16 coordinate range (0–65535) is partitioned as: 16384 buffer + 32768 extent + 16384 buffer = 65536.

### 3.2 Geometry

Geometry is a union of four topology-specific tables:

| Table | Use Cases |
|---|---|
| PointGeometry | Trees, POIs |
| LineGeometry | Highways |
| PolygonGeometry | Surface zones, building footprints |
| MeshGeometry | Terrain |

Each table carries only the fields relevant to its topology. The union discriminator identifies which type is present — no separate topology enum is needed. Multi-part geometry is expressed by the presence of offset arrays on the geometry table.

### 3.3 Features

Each feature has:

- **id**: uint64 (for selection/interaction, 0 = no ID)
- **geometry**: geometry union (one of PointGeometry, LineGeometry, PolygonGeometry, MeshGeometry)
- **properties**: array of `Property` structs, each referencing an entry in `Tile.keys` and `Tile.values`

### 3.4 Geometry Encoding

All four topologies share the same coordinate layout, z encoding, and offset-based multi-part mechanism.

#### Coordinate Layout

All topologies store coordinates as separate `x`, `y`, and `z` arrays (structure-of-arrays layout). All three are required. Vertex count is `x.length`.

- **x**: `[uint16]` — horizontal position, quantized to extent 32768
- **y**: `[uint16]` — vertical position, quantized to extent 32768
- **z**: `[int32]` — elevation in millimeters above WGS84 ellipsoid

The tile proper spans raw values [16384, 49151]. Values in [0, 16383] represent buffer geometry beyond the west/south edge; values in [49152, 65535] represent buffer beyond the east/north edge. The buffer extends 50% of tile width on each side. Mesh vertices (terrain, landmarks) should stay within the tile range [16384, 49151].

#### Elevation (z)

Every vertex has an elevation: int32 millimeters above WGS84 ellipsoid. No dequantization needed — values are direct: `altitude_m = z_value * 0.001`. int32 gives 1mm precision with a ±2,147 km range, covering any location on Earth without per-geometry range parameters. The tile producer resolves terrain-relative source data to ellipsoid heights at build time.

Building extrusion parameters (`height`, `min_height`) are stored as feature properties, not as geometry fields.

#### Multi-part Structure

Composite types use offset arrays (N+1 elements). Each offset array is named after what it partitions into:

| Table | Field | Indexes into | Purpose |
|----------|-------|-------------|---------|
| PointGeometry | — | — | Not used (each element in x/y is a point) |
| LineGeometry | `line_offsets` | x/y (vertices) | Linestring boundaries: linestring i spans vertices `line_offsets[i]` to `line_offsets[i+1]` |
| PolygonGeometry | `ring_offsets` | x/y (vertices) | Ring boundaries: first ring is exterior, subsequent are holes |
| PolygonGeometry | `polygon_offsets` | rings | Multi-polygon: polygon i contains rings `polygon_offsets[i]` to `polygon_offsets[i+1]` |
| MeshGeometry | — | — | Not used; `parts` array defines index ranges per part (see MeshGeometry Fields) |

Offset omission rules:

- **PointGeometry**: no offsets (each x/y/z element is a point)
- **LineGeometry**: single linestring omits `line_offsets`; multi-linestring requires it
- **PolygonGeometry**: single ring (no holes) omits `ring_offsets`; polygon with holes requires `ring_offsets`; multi-polygon requires both `ring_offsets` and `polygon_offsets`
- **MeshGeometry**: no offsets; `parts` array defines sub-ranges (see below)

#### Multi-polygon Example

A feature with two polygons: polygon 0 has an exterior ring (4 vertices) and a hole (3 vertices); polygon 1 has an exterior ring only (5 vertices). Total: 12 vertices, 3 rings, 2 polygons.

```
x: [x0..x3, x4..x6, x7..x11]      ← 12 vertices
y: [y0..y3, y4..y6, y7..y11]
z: [z0..z3, z4..z6, z7..z11]

ring_offsets:    [0, 4, 7, 12]      ← 3 rings → 4 elements (N+1)
  ring 0: vertices 0..3   (polygon 0 exterior)
  ring 1: vertices 4..6   (polygon 0 hole)
  ring 2: vertices 7..11  (polygon 1 exterior)

polygon_offsets: [0, 2, 3]          ← 2 polygons → 3 elements (N+1)
  polygon 0: rings 0..1   (exterior + hole)
  polygon 1: ring 2       (exterior only)
```

#### MeshGeometry Fields

MeshGeometry carries additional fields beyond the shared coordinates:

| Field | Description |
|---|---|
| indices | `[uint32]` triangle indices (required) |
| normals | `[int8]` octahedral-encoded per-vertex normals (nx0, ny0, nx1, ny1, ...), ~1.4 degree precision. Length must equal `2 * x.length`. |
| parts | `[Part]` per-part index range and material; absent → whole mesh is one client-styled draw |
| edge_west/south/east/north | `[uint32]` vertex indices along each tile edge for terrain stitching |

Vertex count is `x.length`. Triangle count is `indices.length / 3`. These values are not stored separately.

A multi-part model (e.g., a landmark with stone base, glass facade, copper roof) concatenates all vertices in x/y/z and all triangle indices in `indices`. Each `Part` defines the index range and material for one visual part of the model.

#### Part

A Part is a contiguous range of triangle indices with an inline material. Each Part maps directly to one GPU draw call. Fields:

- **first_index**: offset into the indices array
- **index_count**: number of indices (triangles = index_count / 3)
- **color**: RGBA base color; `a = 0` means client-styled (no embedded material)
- **roughness**: uint8, 0–255 mapped to 0.0–1.0 (smooth/reflective to rough/matte); ignored when `a = 0`
- **metalness**: uint8, 0–255 mapped to 0.0–1.0 (dielectric to metal); ignored when `a = 0`

Part is a FlatBuffers struct (16 bytes, contiguous in the `parts` vector). When `color.a > 0`, the client uses the embedded material for PBR-lite shading. When `color.a == 0`, the client styles the part based on feature properties (class, subclass).

#### Edge Stitching

Edge index arrays (`edge_west`/`south`/`east`/`north`) on MeshGeometry list vertex indices along each tile edge for seamless terrain stitching between adjacent tiles. Only used by terrain mesh features.

### 3.5 Raster Data

Rasters are regular grids at tile scope, independent from vector layers. Used for blend masks and shader inputs (e.g., terrain noise, surface blending). The `Tile.rasters` field holds zero or more named raster grids.

Each `Raster` has:

- **name**: string identifier
- **width**, **height**: total grid dimensions including buffer pixels
- **buffer**: extra pixels per side for seamless filtering at tile edges
- **channels**: bytes per pixel (1=R, 2=RG, 3=RGB, 4=RGBA; default 1)
- **data**: uint8 pixel values, row-major, bottom-to-top (row 0 = south)

Pixel values are uint8 intensities (0–255 → 0.0–1.0) passed directly to the shader. The `buffer` field specifies extra pixels per side that overlap with neighboring tiles, enabling seamless bilinear filtering without neighbor tile lookups:

```
extent_w = width  - 2 * buffer
extent_h = height - 2 * buffer

lon = lon_west  + ((col - buffer) / extent_w) * (lon_east  - lon_west)
lat = lat_south + ((row - buffer) / extent_h) * (lat_north - lat_south)
```

---

## 4. Properties

### Dictionary Encoding

Properties use dictionary encoding with two tile-level structures:

- **`Tile.keys`**: Property key dictionary. Each entry is a string identifying a property key name used in the tile. The encoder deduplicates keys at tile scope.
- **`Tile.values`**: Property value dictionary. Each entry is a typed `Value` (string, int, double, or bool). The encoder deduplicates values at tile scope.

Each feature stores a **`properties`** array of `Property` structs. Each `Property` is an 8-byte struct with two fields: `key` (index into `Tile.keys`) and `value` (index into `Tile.values`).

Example: a building with `class=residential`, `height=12.5`:

```
Tile.keys:    ["class", "height"]
                k0        k1

Tile.values:  [
  { type: String, string_value: "residential" },  ← v0
  { type: Double, double_value: 12.5 },            ← v1
]

Feature.properties: [
  { key: 0, value: 0 },   ← keys[0]="class",  values[0]="residential"
  { key: 1, value: 1 },   ← keys[1]="height", values[1]=12.5
]
```

### Value Types

| Type | ID | Storage |
|---|---|---|
| String | 0 | FlatBuffers string on `Value` |
| Int | 1 | int64 |
| Double | 2 | float64 |
| Bool | 3 | bool |

All numeric integers use int64. All floating-point values use float64. There are no separate uint or float32 types.

### Reserved Property Keys

These keys have defined semantics across all layers. When present, reserved keys MUST appear in their defined order at the start of `Tile.keys`:

| Index | Key | Type | Description |
|---|---|---|---|
| 0 | class | string | Feature class for style matching |
| 1 | subclass | string | Finer classification |
| 2 | name | string | Display name |
| 3 | height | double | Height in meters; semantics are layer-dependent (absolute above ellipsoid for buildings, relative above ground for vegetation — see layer schema) |
| 4 | min_height | double | Base height in meters above ellipsoid (see layer schema) |
| 5 | level | int | Vertical stacking (-1=tunnel, 0=ground, 1+=elevated) |
| 6 | rank | int | Label priority / importance |

Reserved keys that are unused in a tile MAY be omitted; present keys keep their relative order but indices shift down (e.g., a tile using only `class` and `height` has those at indices 0 and 1 in `keys`). The decoder MUST match reserved keys by string value on first access to determine which are present, then cache the mapping for subsequent O(1) lookups. User-defined keys occupy indices after the last reserved key.

---

## 5. Coordinate System

- **Reference**: WGS84 (EPSG:4326)
- **Tile bounds**: longitude/latitude degrees
- **All geometry x/y**: quantized uint16; extent 32768 with buffer 16384 per side. Tile proper spans raw values [16384, 49151]; values outside are buffer geometry.
- **All geometry z**: int32 millimeters above WGS84 ellipsoid (required, direct value)

One dequantization formula for all topologies:

```
lon = lon_west + ((qx - 16384) / 32768) * (lon_east - lon_west)
lat = lat_south + ((qy - 16384) / 32768) * (lat_north - lat_south)
alt = z * 0.001
```

| Zoom | Tile width | X/Y precision |
|------|-----------|---------------|
| 0 | ~20,000 km | 611 m |
| 5 | ~626 km | 19 m |
| 10 | ~19.6 km | 598 mm |
| 13 | ~2.44 km | 74 mm |
| 15 | ~611 m | 19 mm |
| 16 | ~305 m | 9 mm |

Client reconstructs world coordinates: quantized → geodetic → ECEF (or local tangent plane for rendering). All large-value math in double precision, cast to float32 for GPU only after subtracting camera position.

---

## 6. FlatBuffers Schema

```flatbuffers
namespace arpentry.tiles;
file_identifier "arpt";
file_extension "arpt";

enum PropertyValueType : uint8 {
  String = 0, Int = 1, Double = 2, Bool = 3
}

struct Color {
  r: uint8;
  g: uint8;
  b: uint8;
  a: uint8;
}

struct Part {
  first_index: uint32;          // offset into indices array
  index_count: uint32;          // number of indices (triangles = index_count / 3)
  color: Color;                 // RGBA base color; a=0 → client-styled
  roughness: uint8;             // 0–255 → 0.0–1.0; ignored when a=0
  metalness: uint8;             // 0–255 → 0.0–1.0; ignored when a=0
  // 2 bytes padding (uint32 alignment) — 16 bytes total
}

// --- Per-topology geometry tables ---
// All tables share x/y/z coordinate arrays (SoA layout).
// uint16 coordinates: extent 32768, buffer 16384 per side.
// Tile proper: [16384, 49151]; buffer: [0, 16383] and [49152, 65535].
// z: int32 millimeters above WGS84 ellipsoid.

table PointGeometry {
  x: [uint16] (required);
  y: [uint16] (required);
  z: [int32] (required);
}

table LineGeometry {
  x: [uint16] (required);
  y: [uint16] (required);
  z: [int32] (required);
  line_offsets: [uint32];      // N+1 vertex offsets partitioning into linestrings
}

table PolygonGeometry {
  x: [uint16] (required);
  y: [uint16] (required);
  z: [int32] (required);
  ring_offsets: [uint32];      // N+1 vertex offsets partitioning into rings (exterior first, then holes)
  polygon_offsets: [uint32];  // MultiPolygon: ring offsets partitioning into polygons
}

table MeshGeometry {
  x: [uint16] (required);
  y: [uint16] (required);
  z: [int32] (required);
  indices: [uint32] (required);  // triangle indices
  normals: [int8];               // octahedral int8x2 per-vertex normals
  parts: [Part];                 // per-part index range + material (absent → single client-styled draw)
  edge_west: [uint32];           // terrain edge stitching
  edge_south: [uint32];
  edge_east: [uint32];
  edge_north: [uint32];
}

union Geometry { PointGeometry, LineGeometry, PolygonGeometry, MeshGeometry }

table Value {
  type: PropertyValueType;
  string_value: string;              // when type = String
  int_value: int64;                  // when type = Int
  double_value: float64;             // when type = Double
  bool_value: bool;                  // when type = Bool
}

struct Property {
  key: uint32;                     // index into Tile.keys
  value: uint32;                   // index into Tile.values
}

table Feature {
  id: uint64 = 0;
  geometry: Geometry (required);
  properties: [Property];          // key-value pairs referencing Tile.keys / Tile.values
}

table Layer {
  name: string (required);
  features: [Feature];
}

// --- Raster data ---
// Regular grids at tile scope, independent from vector layers.
// Row-major, bottom-to-top (row 0 = south). Length = width * height * channels.

table Raster {
  name: string (required);        // channel identifier
  width: uint16;                  // total grid width including buffer
  height: uint16;                 // total grid height including buffer
  buffer: uint8;                  // extra pixels per side (0 = no buffer)
  channels: uint8 = 1;           // bytes per pixel (1=R, 2=RG, 3=RGB, 4=RGBA)
  data: [uint8] (required);      // row-major, bottom-to-top, length = width * height * channels
}

// Layers ordered by decode priority (terrain first). See Section 9.
table Tile {
  version: uint16 = 1;            // format version (see Section 7)
  layers: [Layer];
  keys: [string];                   // property key dictionary: deduplicated key names (see Section 4)
  values: [Value];                 // property value dictionary: deduplicated typed values (see Section 4)
  rasters: [Raster];               // named raster grids (see Section 3.5)
}

root_type Tile;
```

---

## 7. File Format

An `.arpt` file is a standard FlatBuffer with file identifier `"arpt"`. There is no custom header — the FlatBuffer starts at byte 0.

- **File identification**: The decoder calls `Tile_identifier_is(buffer)` to verify the 4-byte file identifier (`"arpt"` at offset 4-7, per the FlatBuffers spec).
- **Versioning**: `Tile.version` (uint16, default 1). The decoder MUST reject unknown versions. FlatBuffers schema evolution handles backward-compatible additions without incrementing the version — new optional fields are silently ignored by older decoders.
- **MIME type**: `application/x-arpt`
- **Extension**: `.arpt`
- **URL pattern**: `{base_url}/{level}/{x}/{y}.arpt`

---

## 8. Compression

External compression only - no internal compression within the FlatBuffer:

- **Compression**: Brotli everywhere. Files on disk and over the wire are always Brotli-compressed; `Content-Encoding: br` enables transparent browser decompression. Brotli achieves 70-80% reduction on quantized integer data, and a single codec simplifies the toolchain.
- **Rationale**: FlatBuffers zero-copy requires uncompressed buffers in memory. Quantized integers are highly compressible, so external compression is sufficient.

---

## 9. Layer Schema

The format supports arbitrary named layers. This section defines the reference schema. Zoom ranges shown are for the reference schema; the tiling scheme supports up to level 22 (see Section 2).

### Layer Ordering

Layers in the `layers` vector MUST be ordered by decode priority (first decoded, first rendered). For the reference schema, the required order is:

1. terrain
2. surface
3. highway
4. tree
5. building
6. poi

This ordering enables progressive decode: the client can read `layers[0]` (terrain), submit it to the GPU for an initial frame, and decode remaining layers across subsequent frames. FlatBuffers' lazy access means untouched layers incur no decode cost. Layers not present in a tile are simply omitted (indices shift down); the layer name field disambiguates.

### terrain

Ground elevation. One Mesh feature per tile.

- **Geometry**: Mesh
- **Zoom range**: 0-16
- **Properties**: none

### surface

Physical ground cover. Polygon fills styled by class.

- **Geometry**: Polygon
- **Zoom range**: 0-16
- **Properties**:

| Key | Type | Values |
|---|---|---|
| class | string | `grass`, `forest`, `water`, `rock`, `sand`, `ice` |

### highway

Roads and paths.

- **Geometry**: Line
- **Zoom range**: 8-16
- **Properties**:

| Key | Type | Values |
|---|---|---|
| class | string | `motorway`, `primary`, `secondary`, `local` |
| name | string | Street name |

### tree

Individual trees.

- **Geometry**: Point
- **Zoom range**: 14-16
- **Properties**:

| Key | Type | Values |
|---|---|---|
| class | string | `broadleaf`, `conifer` |
| height | double | Height in meters above ground |

### building

Extruded building footprints. The client computes extrusion from `min_height` (base) to `height` (top).

- **Geometry**: Polygon
- **Zoom range**: 13-16
- **Properties**:

| Key | Type | Values |
|---|---|---|
| class | string | `residential`, `commercial`, `industrial`, `civic` |
| height | double | Top in meters above ellipsoid |
| min_height | double | Base in meters above ellipsoid |

### poi

Points of interest for labels and icons.

- **Geometry**: Point
- **Zoom range**: 10-16
- **Properties**:

| Key | Type | Values |
|---|---|---|
| class | string | `restaurant`, `hotel`, `shop`, `transit`, `school`, `hospital` |
| name | string | Display name |
| rank | int | Label priority (lower = more important) |

---

## 10. Styling Contract

The format provides data for styling. For PointGeometry, LineGeometry, and PolygonGeometry, the client determines all visual appearance based on:

1. **Layer name** — which style rules apply
2. **Geometry type** — which union member is present determines the rendering method (fill, line, extrusion, mesh)
3. **Feature class/subclass** — primary style discriminator
4. **Numeric properties** — data-driven styling (height → extrusion, width → line thickness)
5. **Feature ID** — interaction (hover, click, selection)

For MeshGeometry, each Part carries an inline material (color, roughness, metalness). When `color.a > 0`, the client uses it directly for PBR-lite shading. When `color.a == 0`, the client styles the part based on feature properties, as with other topologies. When no `parts` array is present, the entire mesh is one client-styled draw call.

Raster data is consumed as shader inputs directly (0–255 → 0.0–1.0), keyed by raster name.

Style specification is defined in a separate document.

---

## 11. Performance Targets

### Tile Sizes

| Content | Uncompressed | Brotli |
|---|---|---|
| Terrain only (level 10) | 50-150 KB | 15-50 KB |
| Vector features (buildings) | 20-100 KB | 5-30 KB |
| Combined urban tile | 100-500 KB | 30-150 KB |
| Dense urban max | 500 KB - 1 MB | 150-300 KB |

95th percentile target: < 600 KB uncompressed, < 200 KB compressed (including rasters).

### Decode Times (mid-range 2023 device)

| Operation | Target |
|---|---|
| Open tile, read layer list | < 1 us (zero-copy) |
| Iterate 1000 features | < 100 us |
| Decode 4000-vertex mesh | < 200 us |
| Full tile decode + GPU upload | < 5 ms |

### Memory Budget

- Per tile in-memory: 100-500 KB (raw FlatBuffer)
- 50-200 tiles loaded simultaneously
- Total tile memory: 10-50 MB
- Total GPU memory for tiles: 5-30 MB
