#ifndef ARPENTRY_GEN_TOWN_H
#define ARPENTRY_GEN_TOWN_H

#include "coords.h"

#include <stdbool.h>

/* Town value indices into the tile-scope value dictionary (continued from
 * SURFACE_VAL_* in surface.h). */
#define TOWN_VAL_PRIMARY     7
#define TOWN_VAL_RESIDENTIAL 8
#define TOWN_VAL_BUILDING    9
#define TOWN_VAL_H5          10
#define TOWN_VAL_H8          11
#define TOWN_VAL_H10         12
#define TOWN_VAL_H12         13
#define TOWN_VAL_H15         14

/* Town key indices into the tile-scope key dictionary */
#define TOWN_KEY_HEIGHT 1

/* Road segment (two endpoints in degrees). */
typedef struct {
    double lon1, lat1;
    double lon2, lat2;
    int cls;
} town_road;

/* Building footprint (center + dimensions in degrees/meters). */
typedef struct {
    double lon, lat;
    double w_m, h_m;
    int cls;
    int height_val;
} town_building;

/* Return true if the tile bounds overlap the procedural town area. */
bool town_overlaps(arpt_bounds bounds);

int town_road_count(void);
const town_road *town_get_roads(void);

int town_building_count(void);
const town_building *town_get_buildings(void);

#endif /* ARPENTRY_GEN_TOWN_H */
