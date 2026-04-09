#ifndef ARPENTRY_TILE_DECODE_H
#define ARPENTRY_TILE_DECODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Default layer name for terrain (auto-generated mesh). */
#define ARPT_LAYER_TERRAIN_NAME  "terrain"

/**
 * Zero-copy terrain mesh data extracted from a FlatBuffer tile.
 * All pointers point directly into the FlatBuffer — valid only while
 * the buffer is alive.
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

#define ARPT_MAX_CLASSES 64

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer */
    const int32_t *z;      /* elevation in millimeters (NULL for surface) */
    size_t vertex_count;
    uint8_t cls;      /* index into style class registry; 0 = unknown */
    uint16_t poly_id; /* polygon ID: rings sharing a poly_id belong to
                         the same polygon (exterior + holes) */
    int32_t height_m; /* building height in meters (0 for surface polygons) */
} arpt_surface_polygon;

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

/* Highway decoding (LineGeometry) */

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer */
    size_t vertex_count;
    uint8_t cls; /* index into style class registry; 0 = unknown */
} arpt_highway_line;

typedef struct {
    arpt_highway_line *lines; /* malloc'd array */
    size_t count;
} arpt_highway_data;

/**
 * Extract highway lines from a named layer in a verified FlatBuffer tile.
 * Extracts LineGeometry features with their class.
 */
bool arpt_decode_highways(const void *flatbuf, size_t size,
                          const char *layer_name,
                          const char (*class_names)[32], int class_count,
                          arpt_highway_data *out);

void arpt_highway_data_free(arpt_highway_data *data);

/* Building decoding (PolygonGeometry, same struct as surface) */

/**
 * Extract building footprints from a named layer in a verified FlatBuffer tile.
 * Extracts PolygonGeometry features with their class and height.
 */
bool arpt_decode_buildings(const void *flatbuf, size_t size,
                           const char *layer_name,
                           const char (*class_names)[32], int class_count,
                           arpt_surface_data *out);

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
