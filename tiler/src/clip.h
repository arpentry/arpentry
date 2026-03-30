/* Geometry clipping for tile assignment. */

#ifndef ARPT_CLIP_H
#define ARPT_CLIP_H

#include "geom.h"

/* Equirectangular tile bounds in WGS84 degrees for tile (z, x, y).
   Grid: 2^(z+1) columns × 2^z rows, y=0 at south. */
arpt_bounds arpt_tile_bounds(int z, int tx, int ty);

/* Callback invoked for each (tile, clipped geometry) pair. */
typedef void (*arpt_tile_cb)(int z, int x, int y,
                             const arpt_geom *clipped, void *ctx);

/* Assign a geometry to tiles at the given zoom level. */
void arpt_assign_tiles(const arpt_geom *geom, int zoom,
                       arpt_tile_cb cb, void *ctx);

/* Process a geometry across a zoom range: simplify incrementally from
   finest to coarsest, skip sub-pixel features and those exceeding
   max_span tiles in either dimension, then assign to tiles via the
   callback. */
void arpt_process_feature_zooms(const arpt_geom *geom, const double bbox[4],
                                int min_zoom, int max_zoom, int max_span,
                                arpt_tile_cb cb, void *ctx);

#endif
