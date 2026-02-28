#ifndef ARPENTRY_GEN_SURFACE_H
#define ARPENTRY_GEN_SURFACE_H

#include "coords.h"

#include <stdint.h>

/* Surface grid resolution */
#define SURFACE_GRID 64  /* 64x64 marching squares grid (matches terrain) */
#define SURFACE_BUFFER 8 /* extra cells of buffer on each side */
#define SURFACE_TOTAL (SURFACE_GRID + 2 * SURFACE_BUFFER) /* 80 */
#define SURFACE_VERTS (SURFACE_TOTAL + 1) /* 81x81 classification vertices */

/* Surface class indices into the tile-scope value dictionary */
#define SURFACE_VAL_WATER     0
#define SURFACE_VAL_DESERT    1
#define SURFACE_VAL_FOREST    2
#define SURFACE_VAL_GRASSLAND 3
#define SURFACE_VAL_CROPLAND  4
#define SURFACE_VAL_SHRUB     5
#define SURFACE_VAL_ICE       6

/* Marching squares patch: a single polygon from one grid cell */
typedef struct {
    uint16_t x[9], y[9];
    int count, cls;
} ms_patch;

/* Classify surface type at each vertex of the marching squares grid. */
void classify_surface(arpt_bounds bounds, double lon_span, double lat_span,
                      int *vert_class);

/* Generate surface polygon patches via marching squares.
 * Returns the number of patches written. */
int generate_surface_patches(const int *vert_class, ms_patch *patches);

#endif /* ARPENTRY_GEN_SURFACE_H */
