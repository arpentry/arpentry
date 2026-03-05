#include "poi.h"

#include <stdbool.h>

/*
 * Hardcoded points of interest near the procedural town at (0, 0).
 * Coordinates are in degrees; all POIs lie within the town area.
 */

/* POI area bounding box (slightly larger than the town core) */
#define POI_WEST  (-0.010)
#define POI_EAST    0.010
#define POI_SOUTH (-0.010)
#define POI_NORTH   0.010

static const poi_point pois[] = {
    { 0.0000,  0.0000, "Town Hall"},
    {-0.0030,  0.0025, "Library"},
    { 0.0035, -0.0020, "Market"},
    {-0.0050, -0.0040, "Park"},
    { 0.0040,  0.0035, "School"},
    { 0.0020, -0.0050, "Station"},
    {-0.0045,  0.0050, "Hospital"},
    { 0.0055,  0.0010, "Museum"},
};

#define N_POIS ((int)(sizeof(pois) / sizeof(pois[0])))

bool poi_overlaps(arpt_bounds bounds) {
    return bounds.east > POI_WEST && bounds.west < POI_EAST &&
           bounds.north > POI_SOUTH && bounds.south < POI_NORTH;
}

int poi_count(void) {
    return N_POIS;
}

const poi_point *poi_get_points(void) {
    return pois;
}
