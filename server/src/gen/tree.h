#ifndef ARPENTRY_GEN_TREE_H
#define ARPENTRY_GEN_TREE_H

#include "coords.h"

/* Tree value index into the tile-scope value dictionary (continued from
 * TOWN_VAL_H15 = 14 in town.h). */
#define TREE_VAL_TREE 15
#define TREE_GRID_MAX 4096 /* max trees per tile */

/* A single tree position in geodetic coordinates. */
typedef struct {
    double lon, lat;
} tree_point;

/* Generate tree positions within the given tile bounds.
 * Trees are placed on a stable global grid with deterministic jitter.
 * Returns the number of trees written (at most max_count). */
int generate_trees(arpt_bounds bounds, tree_point *out, int max_count);

#endif /* ARPENTRY_GEN_TREE_H */
