#ifndef ARPENTRY_TILE_DECODE_H
#define ARPENTRY_TILE_DECODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

typedef enum {
    ARPT_SURFACE_UNKNOWN     = 0,
    ARPT_SURFACE_WATER,
    ARPT_SURFACE_DESERT,
    ARPT_SURFACE_FOREST,
    ARPT_SURFACE_GRASSLAND,
    ARPT_SURFACE_CROPLAND,
    ARPT_SURFACE_SHRUB,
    ARPT_SURFACE_ICE,
    ARPT_SURFACE_PRIMARY,     /* primary road */
    ARPT_SURFACE_RESIDENTIAL, /* residential road */
    ARPT_SURFACE_BUILDING,    /* building footprint */
} arpt_surface_class;

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer */
    const int32_t *z;      /* elevation in millimeters (NULL for surface) */
    size_t vertex_count;
    arpt_surface_class cls;
    int32_t height_m; /* building height in meters (0 for surface polygons) */
} arpt_surface_polygon;

typedef struct {
    arpt_surface_polygon *polygons; /* malloc'd array */
    size_t count;
} arpt_surface_data;

/**
 * Extract surface polygons from a verified FlatBuffer tile.
 *
 * Finds the "surface" layer, resolves the "class" property key,
 * and extracts PolygonGeometry features with their class.
 *
 * Returns true even if no surface layer is found (count=0).
 * Returns false only on allocation failure.
 */
bool arpt_decode_surface(const void *flatbuf, size_t size,
                         arpt_surface_data *out);

void arpt_surface_data_free(arpt_surface_data *data);

/* Highway decoding (LineGeometry) */

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer */
    size_t vertex_count;
    arpt_surface_class cls;
} arpt_highway_line;

typedef struct {
    arpt_highway_line *lines; /* malloc'd array */
    size_t count;
} arpt_highway_data;

/**
 * Extract highway lines from a verified FlatBuffer tile.
 * Finds the "highway" layer, extracts LineGeometry features.
 */
bool arpt_decode_highways(const void *flatbuf, size_t size,
                          arpt_highway_data *out);

void arpt_highway_data_free(arpt_highway_data *data);

/* Building decoding (PolygonGeometry, same struct as surface) */

/**
 * Extract building footprints from a verified FlatBuffer tile.
 * Finds the "building" layer, extracts PolygonGeometry features.
 */
bool arpt_decode_buildings(const void *flatbuf, size_t size,
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
                       const char *const *class_names, int class_count,
                       arpt_tree_data *out);

void arpt_tree_data_free(arpt_tree_data *data);

#endif /* ARPENTRY_TILE_DECODE_H */
