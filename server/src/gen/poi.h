#ifndef ARPENTRY_GEN_POI_H
#define ARPENTRY_GEN_POI_H

#include "coords.h"

#include <stdbool.h>

/* POI key index into the tile-scope key dictionary */
#define POI_KEY_NAME 2

/* POI value indices into the tile-scope value dictionary (continued from
 * TREE_VAL_BIRCH = 17 in tree.h). */
#define POI_VAL_POI       18 /* class value for "poi" */
#define POI_VAL_NAME_BASE 19 /* first name string index */

#define POI_MAX 16

/* A single point of interest in geodetic coordinates. */
typedef struct {
    double lon, lat;
    const char *name;
} poi_point;

/* Return true if the tile bounds overlap the POI area. */
bool poi_overlaps(arpt_bounds bounds);

int poi_count(void);
const poi_point *poi_get_points(void);

#endif /* ARPENTRY_GEN_POI_H */
