/* Minimal GeoTIFF DEM reader for single-band int16 elevation grids. */

#ifndef ARPT_DEM_H
#define ARPT_DEM_H

#include <stdint.h>

typedef struct arpt_dem arpt_dem;

/* Open a GeoTIFF DEM file (single-band int16, uncompressed).
   Returns NULL on failure. */
arpt_dem *arpt_dem_open(const char *path);

/* Sample elevation in meters at the given lon/lat (degrees).
   Uses bilinear interpolation. Returns 0 for out-of-bounds. */
double arpt_dem_sample(const arpt_dem *dem, double lon, double lat);

/* Free DEM resources. */
void arpt_dem_free(arpt_dem *dem);

#endif
