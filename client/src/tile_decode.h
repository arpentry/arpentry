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
    const uint16_t *x;       /* horizontal positions */
    const uint16_t *y;       /* vertical positions */
    const int32_t  *z;       /* elevation in millimeters */
    const int8_t   *normals; /* octahedral int8x2, NULL if absent */
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

#endif /* ARPENTRY_TILE_DECODE_H */
