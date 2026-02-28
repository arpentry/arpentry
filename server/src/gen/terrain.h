#ifndef ARPENTRY_GEN_TERRAIN_H
#define ARPENTRY_GEN_TERRAIN_H

#include "coords.h"

#include <stdint.h>

/* Terrain grid resolution */
#define TERRAIN_GRID 64
#define TERRAIN_VERTS (TERRAIN_GRID + 1) /* 65x65 = 4225 vertices */

/* Compute terrain elevation in meters at a geodetic point.
 * Positive = land, negative = ocean.  Deterministic. */
double terrain_elevation(double lon_deg, double lat_deg);

/* Compute moisture in [0, 1] at a geodetic point.  Deterministic. */
double terrain_moisture(double lon_deg, double lat_deg);

/* Build padded elevation grid.
 * Returns (TERRAIN_VERTS+2)^2 doubles, or NULL on failure.
 * Caller frees the returned buffer. */
double *build_elevation_grid(arpt_bounds bounds, double cell_lon,
                             double cell_lat);

/* Build vertex arrays (positions + normals) from the padded elevation grid. */
void build_vertices(arpt_bounds bounds, double cell_lon, double cell_lat,
                    double cell_w_m, double cell_h_m, const double *elev,
                    int pad_w, uint16_t *vx, uint16_t *vy, int32_t *vz,
                    int8_t *normals);

/* Build triangle indices for the terrain grid. */
void build_indices(uint32_t *indices);

#endif /* ARPENTRY_GEN_TERRAIN_H */
