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
    ARPT_SURFACE_UNKNOWN = 0,
    ARPT_SURFACE_GRASS,
    ARPT_SURFACE_FOREST,
    ARPT_SURFACE_SAND,
} arpt_surface_class;

typedef struct {
    const uint16_t *x, *y; /* zero-copy into FlatBuffer */
    size_t vertex_count;
    arpt_surface_class cls;
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

#endif /* ARPENTRY_TILE_DECODE_H */
