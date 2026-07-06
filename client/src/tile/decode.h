#ifndef ARPENTRY_TILE_DECODE_H
#define ARPENTRY_TILE_DECODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Default layer name for terrain (auto-generated mesh). */
#define ARPT_LAYER_TERRAIN_NAME  "terrain"

/**
 * Zero-copy terrain mesh extracted from a FlatBuffer tile.
 *
 * LIFETIME: the x/y/z/indices/normals pointers alias into the FlatBuffer
 * passed to arpt_decode_terrain. They are only valid until that buffer is
 * freed. The typical consumption pattern is:
 *     decode → prepare (copies into renderer primitives) → free flatbuf.
 * Never read these pointers after the backing buffer has been freed.
 */
typedef struct {
    const uint16_t *x;     /* horizontal positions */
    const uint16_t *y;     /* vertical positions */
    const int32_t *z;      /* elevation in millimeters */
    const int8_t *normals; /* octahedral int8x2, NULL if absent */
    size_t vertex_count;
    const uint32_t *indices;
    size_t index_count;
} arpt_terrain_mesh;

/**
 * Extract the terrain mesh from a verified FlatBuffer tile.
 *
 * Finds the "terrain" layer, extracts the first MeshGeometry feature's
 * arrays via zero-copy FlatCC reader API.
 *
 * Returns false if no terrain layer, no features, or wrong geometry type.
 */
bool arpt_decode_terrain(const void *flatbuf, size_t size,
                         arpt_terrain_mesh *out);

/* Surface decoding */

/* Class-index registry capacity.  The decoded `cls` fields are uint8_t,
 * so 256 is the hard upper bound; reserving the full range keeps the
 * registry from filling up as styles grow. */
#define ARPT_MAX_CLASSES 256

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer (see arpt_surface_data) */
    const int32_t *z;      /* elevation in millimeters (NULL for surface) */
    size_t vertex_count;
    uint8_t cls;      /* index into style class registry; 0 = unknown */
    uint16_t poly_id; /* polygon ID: rings sharing a poly_id belong to
                         the same polygon (exterior + holes) */
} arpt_surface_polygon;

/**
 * Polygon features decoded from one tile layer.
 *
 * LIFETIME: `polygons` is a malloc'd array owned by this struct, but the
 * `x`/`y`/`z` pointers inside each entry alias into the FlatBuffer passed
 * to the decode call. The FlatBuffer must remain alive until arpt_prepare_*
 * has consumed these pointers. arpt_surface_data_free only releases the
 * `polygons` array — it does not own the FlatBuffer.
 */
typedef struct {
    arpt_surface_polygon *polygons; /* malloc'd array */
    size_t count;
} arpt_surface_data;

/**
 * Extract surface polygons from a named layer in a verified FlatBuffer tile.
 *
 * Resolves the "class" property key and extracts PolygonGeometry features.
 *
 * Returns true even if the layer is not found (count=0).
 * Returns false only on allocation failure.
 */
bool arpt_decode_surface_layer(const void *flatbuf, size_t size,
                               const char *layer_name,
                               const char (*class_names)[32], int class_count,
                               arpt_surface_data *out);

void arpt_surface_data_free(arpt_surface_data *data);

/* Line decoding (LineGeometry) */

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer (see arpt_line_data) */
    const int32_t *z;      /* per-vertex road elevation (mm) the tiler baked from
                              the terrain surface, or NULL when the road is flat
                              (DEM-less). The client strokes the road at these
                              heights instead of sampling terrain. */
    size_t vertex_count;
    uint8_t cls;   /* index into style class registry; 0 = unknown */
    float width_m; /* physical carriageway width in metres the tiler baked from
                      its engineering priors (the same numbers that size the
                      bridge decks), or 0 when absent (non-drivable classes).
                      Close zooms stroke the road at this true width so it
                      meets the structures edge-to-edge. */
} arpt_line_feature;

/**
 * Line features decoded from one tile layer.
 *
 * LIFETIME: `lines` is a malloc'd array owned by this struct, but each
 * entry's `x`/`y` pointers alias into the FlatBuffer passed to
 * arpt_decode_lines. The FlatBuffer must remain alive until arpt_prepare_lines
 * has copied the coordinates into the renderer primitives. See the
 * "zero-copy window" block in tile/manager.c for the exact ordering.
 * arpt_line_data_free only releases the `lines` array.
 */
typedef struct {
    arpt_line_feature *lines; /* malloc'd array */
    size_t count;
} arpt_line_data;

/**
 * Extract line features from a named layer in a verified FlatBuffer tile.
 * Extracts LineGeometry features with their class.
 */
bool arpt_decode_lines(const void *flatbuf, size_t size,
                       const char *layer_name,
                       const char (*class_names)[32], int class_count,
                       arpt_line_data *out);

void arpt_line_data_free(arpt_line_data *data);

/* Line label decoding (LineGeometry with name property) */

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer (see arpt_line_label_data) */
    size_t vertex_count;
    char name[64]; /* copied from value dictionary */
} arpt_line_label_feature;

/**
 * Named line features decoded from one tile layer (street labels).
 *
 * LIFETIME: `features` is a malloc'd array owned by this struct, but each
 * entry's `x`/`y` pointers alias into the FlatBuffer passed to
 * arpt_decode_line_labels. The FlatBuffer must remain alive until
 * arpt_prepare_line_labels has copied the coordinates.
 */
typedef struct {
    arpt_line_label_feature *features; /* malloc'd array */
    size_t count;
} arpt_line_label_data;

/**
 * Extract named line features from a layer in a verified FlatBuffer tile.
 * Extracts LineGeometry features that carry a non-empty "name" string;
 * nameless lines are skipped. Each part of a multi-line becomes one entry.
 */
bool arpt_decode_line_labels(const void *flatbuf, size_t size,
                             const char *layer_name,
                             arpt_line_label_data *out);

void arpt_line_label_data_free(arpt_line_label_data *data);

/* Building meshes are decoded via arpt_decode_building_mesh in prepare.h. */

/* Tree decoding (PointGeometry) */

typedef struct {
    uint16_t qx, qy;
    int32_t z;
    uint8_t model_index; /* index into style tree_styles array */
    uint32_t id;         /* stable ID for deterministic randomness */
} arpt_tree_point;

typedef struct {
    arpt_tree_point *points; /* malloc'd array */
    size_t count;
} arpt_tree_data;

/**
 * Extract tree point positions from a verified FlatBuffer tile.
 * Finds the "tree" layer, extracts PointGeometry features.
 * class_names is an array of class_count class name strings; each tree's
 * model_index is set to the matching index, or 0 if no match.
 */
bool arpt_decode_trees(const void *flatbuf, size_t size,
                       const char *layer_name,
                       const char *const *class_names, int class_count,
                       arpt_tree_data *out);

void arpt_tree_data_free(arpt_tree_data *data);

/* POI decoding (PointGeometry with name property) */

typedef struct {
    uint16_t qx, qy;
    int32_t z;
    char name[64]; /* copied from value dictionary */
    char icon[32]; /* Maki icon name (e.g. "hospital") */
} arpt_poi_point;

typedef struct {
    arpt_poi_point *points; /* malloc'd array */
    size_t count;
} arpt_poi_data;

/**
 * Extract POI points from a verified FlatBuffer tile.
 * Finds the "poi" layer, extracts PointGeometry features with "name" string.
 */
bool arpt_decode_pois(const void *flatbuf, size_t size,
                      const char *layer_name,
                      arpt_poi_data *out);

void arpt_poi_data_free(arpt_poi_data *data);

#endif /* ARPENTRY_TILE_DECODE_H */
