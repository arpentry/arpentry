#include "tree.h"
#include "terrain.h"
#include "noise.h"

/* Biome thresholds (same as surface.c) */
#define BIOME_ELEV_ICE  3000.0
#define BIOME_ELEV_MID  1500.0
#define BIOME_ELEV_LOW   400.0
#define BIOME_MOIST_WET    0.55

/* Grid resolution for candidate placement */
#define TREE_GRID 32

/* Noise frequency for jitter and density */
#define JITTER_FREQ  50.0
#define DENSITY_FREQ 12.0
#define DENSITY_THRESHOLD 0.1

/* Return true if the biome at (lon, lat) is forest. */
static int is_forest(double lon, double lat) {
    double e = terrain_elevation(lon, lat);
    if (e <= 0.0) return 0;

    double m = terrain_moisture(lon, lat);
    if (m <= BIOME_MOIST_WET) return 0;

    if (e > BIOME_ELEV_ICE) return 0;
    /* Mid-high, mid, or low elevation with wet moisture => forest */
    return 1;
}

int generate_trees(arpt_bounds bounds, tree_point *out, int max_count) {
    double lon_span = bounds.east - bounds.west;
    double lat_span = bounds.north - bounds.south;
    double cell_lon = lon_span / TREE_GRID;
    double cell_lat = lat_span / TREE_GRID;
    int count = 0;

    for (int r = 0; r < TREE_GRID && count < max_count; r++) {
        for (int c = 0; c < TREE_GRID && count < max_count; c++) {
            /* Cell center */
            double lon = bounds.west + ((double)c + 0.5) * cell_lon;
            double lat = bounds.south + ((double)r + 0.5) * cell_lat;

            /* Deterministic jitter using simplex noise */
            double jx = arpt_simplex2(lon * JITTER_FREQ, lat * JITTER_FREQ);
            double jy = arpt_simplex2(lat * JITTER_FREQ + 100.0,
                                      lon * JITTER_FREQ + 100.0);
            lon += jx * cell_lon * 0.4;
            lat += jy * cell_lat * 0.4;

            /* Density noise pass */
            double d = arpt_simplex2(lon * DENSITY_FREQ, lat * DENSITY_FREQ);
            if (d < DENSITY_THRESHOLD) continue;

            /* Biome check */
            if (!is_forest(lon, lat)) continue;

            out[count].lon = lon;
            out[count].lat = lat;
            count++;
        }
    }

    return count;
}
