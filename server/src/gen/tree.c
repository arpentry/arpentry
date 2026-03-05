#include "tree.h"
#include "terrain.h"
#include "noise.h"
#include <math.h>

/* Fixed global grid spacing in degrees (~55 m at equator). */
#define CELL_DEG 0.0005

/* Noise frequency for jitter. */
#define JITTER_FREQ 50.0

/* Trees only within this radius (degrees) of the town center (0,0). */
#define TREE_RADIUS_DEG 0.15

int generate_trees(arpt_bounds bounds, tree_point *out, int max_count) {
    /* Clamp iteration to the tree area */
    double lo_lon = fmax(bounds.west,  -TREE_RADIUS_DEG);
    double hi_lon = fmin(bounds.east,   TREE_RADIUS_DEG);
    double lo_lat = fmax(bounds.south, -TREE_RADIUS_DEG);
    double hi_lat = fmin(bounds.north,  TREE_RADIUS_DEG);

    /* No overlap with tree area */
    if (lo_lon >= hi_lon || lo_lat >= hi_lat) return 0;

    /* Snap to the global grid */
    int c0 = (int)floor(lo_lon / CELL_DEG);
    int c1 = (int)ceil(hi_lon / CELL_DEG);
    int r0 = (int)floor(lo_lat / CELL_DEG);
    int r1 = (int)ceil(hi_lat / CELL_DEG);

    int count = 0;

    for (int r = r0; r <= r1 && count < max_count; r++) {
        for (int c = c0; c <= c1 && count < max_count; c++) {
            double lon = ((double)c + 0.5) * CELL_DEG;
            double lat = ((double)r + 0.5) * CELL_DEG;

            /* Only within radius */
            double d2 = lon * lon + lat * lat;
            if (d2 > TREE_RADIUS_DEG * TREE_RADIUS_DEG) continue;

            /* Skip water */
            if (terrain_elevation(lon, lat) <= 0.0) continue;

            /* Deterministic jitter using simplex noise */
            double jx = arpt_simplex2(lon * JITTER_FREQ, lat * JITTER_FREQ);
            double jy = arpt_simplex2(lat * JITTER_FREQ + 100.0,
                                      lon * JITTER_FREQ + 100.0);
            lon += jx * CELL_DEG * 0.4;
            lat += jy * CELL_DEG * 0.4;

            /* Deterministic tree type from position hash */
            uint32_t th = (uint32_t)(c * 73856093) ^ (uint32_t)(r * 19349663);
            int rem = (int)(th % 3);
            tree_type tt = (rem == 0) ? TREE_TYPE_OAK
                         : (rem == 1) ? TREE_TYPE_PINE
                                      : TREE_TYPE_BIRCH;

            out[count].lon = lon;
            out[count].lat = lat;
            out[count].type = tt;
            out[count].id = ((uint64_t)(uint32_t)c << 32) |
                            (uint64_t)(uint32_t)r;
            count++;
        }
    }

    return count;
}
