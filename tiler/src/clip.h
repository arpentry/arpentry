/* Geometry clipping for tile assignment. */

#ifndef ARPT_CLIP_H
#define ARPT_CLIP_H

#include "wkb.h"

/* Bounding box for clipping. */
typedef struct {
    double min_x, min_y, max_x, max_y;
} arpt_bounds;

/* Equirectangular tile bounds in WGS84 degrees for tile (z, x, y).
   Grid: 2^(z+1) columns × 2^z rows, y=0 at south. */
arpt_bounds arpt_tile_bounds(int z, int tx, int ty);

/* Callback invoked for each (tile, clipped geometry) pair. */
typedef void (*arpt_tile_cb)(int z, int x, int y,
                             const arpt_geom *clipped, void *ctx);

/* Assign a geometry to tiles at the given zoom level. */
void arpt_assign_tiles(const arpt_geom *geom, int zoom,
                       arpt_tile_cb cb, void *ctx);

#endif
