/* FlatBuffer tile assembly. */

#ifndef ARPT_TILE_BUILD_H
#define ARPT_TILE_BUILD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "geom.h"

typedef struct arpt_tile_builder arpt_tile_builder;
typedef struct arpt_dem arpt_dem;

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
