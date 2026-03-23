/* FlatBuffer tile assembly. */

#ifndef ARPT_TILE_BUILD_H
#define ARPT_TILE_BUILD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "clip.h"
#include "dem.h"
#include "wkb.h"

typedef struct arpt_tile_builder arpt_tile_builder;

/* A feature to be added to a tile. */
typedef struct {
    uint32_t     layer;
    const arpt_geom *geom;
    const char  *const *prop_keys;
    const char  *const *prop_vals;
    uint32_t     n_props;
} arpt_feature;

/* Create a tile builder for the given tile bounds.
   dem may be NULL for flat terrain. */
arpt_tile_builder *arpt_tile_builder_create(arpt_bounds bounds,
                                            const arpt_dem *dem);

/* Add a feature to the tile. */
bool arpt_tile_builder_add_feature(arpt_tile_builder *b,
                                   const arpt_feature *feat);

/* Finish building. Returns Brotli-compressed .arpt data. Caller frees. */
void *arpt_tile_builder_finish(arpt_tile_builder *b, size_t *out_size);

/* Free the builder. */
void arpt_tile_builder_free(arpt_tile_builder *b);

#endif
