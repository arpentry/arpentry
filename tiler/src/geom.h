/* Core geometry types shared across the tiler. */

#ifndef ARPT_GEOM_H
#define ARPT_GEOM_H

#include <stdint.h>

/* Axis-aligned bounding box in geographic coordinates. */
typedef struct {
    double min_x, min_y, max_x, max_y;
} arpt_bounds;

/* Parsed geometry in SoA layout.
 *
 * Multi-types are flattened at parse time:
 *   MultiPoint      → type 1 with n_coords > 1
 *   MultiLineString → type 2 with offsets separating sub-lines
 *   MultiPolygon    → type 3 with all rings in offsets (even-odd fill)
 *
 * Downstream code only sees types 1 (Point), 2 (LineString), 3 (Polygon). */
typedef struct {
    uint32_t type;       /* geometry type: 1=Point, 2=LineString, 3=Polygon */
    double  *x;          /* x coordinates */
    double  *y;          /* y coordinates */
    double  *z;          /* z coordinates (NULL if 2D) */
    uint32_t n_coords;   /* number of coordinates */
    uint32_t *offsets;   /* ring/line offsets (N+1 sentinel style) */
    uint32_t n_offsets;  /* number of offsets */
} arpt_geom;

/* A feature: geometry + properties + layer assignment. */
typedef struct {
    uint32_t     layer;
    const arpt_geom *geom;
    const char  *const *prop_keys;
    const char  *const *prop_vals;
    uint32_t     n_props;
} arpt_feature;

/* Compute the bounding box of a geometry: bbox[0]=min_x, bbox[1]=min_y,
   bbox[2]=max_x, bbox[3]=max_y. Geometry must have n_coords > 0. */
void arpt_geom_bbox(const arpt_geom *g, double bbox[4]);

/* Free memory owned by an arpt_geom. */
void arpt_geom_free(arpt_geom *g);

#endif
