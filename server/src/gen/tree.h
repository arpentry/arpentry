#ifndef ARPENTRY_GEN_TREE_H
#define ARPENTRY_GEN_TREE_H

#include "coords.h"
#include <stdint.h>

/* Tree value indices into the tile-scope value dictionary (continued from
 * TOWN_VAL_H15 = 14 in town.h). */
#define TREE_VAL_OAK   15
#define TREE_VAL_PINE  16
#define TREE_VAL_BIRCH 17
#define TREE_GRID_MAX 4096 /* max trees per tile */

/* Tree type enum (maps 1:1 to TREE_VAL_* offsets). */
typedef enum {
    TREE_TYPE_OAK   = 0,
    TREE_TYPE_PINE  = 1,
    TREE_TYPE_BIRCH = 2,
} tree_type;

/* A single tree position in geodetic coordinates with type. */
typedef struct {
    double lon, lat;
    tree_type type;
    uint64_t id;  /* stable ID from global grid cell hash */
} tree_point;

/* Generate tree positions within the given tile bounds.
 * Trees are placed on a stable global grid with deterministic jitter.
 * Returns the number of trees written (at most max_count). */
int generate_trees(arpt_bounds bounds, tree_point *out, int max_count);

#endif /* ARPENTRY_GEN_TREE_H */
