#include "coords.h"

arpt_bounds arpt_tile_bounds(int level, int x, int y) {
    double lon_span = 360.0 / (double)(1u << (level + 1));
    double lat_span = 180.0 / (double)(1u << level);
    arpt_bounds b;
    b.west = -180.0 + x * lon_span;
    b.south = -90.0 + y * lat_span;
    b.east = b.west + lon_span;
    b.north = b.south + lat_span;
    return b;
}
