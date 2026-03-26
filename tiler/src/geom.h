/* Core geometry types shared across the tiler. */

#ifndef ARPT_GEOM_H
#define ARPT_GEOM_H

#include <stdint.h>

/* Axis-aligned bounding box in geographic coordinates. */
typedef struct {
    double min_x, min_y, max_x, max_y;
} arpt_bounds;

/* Parsed geometry in SoA layout. */
typedef struct {
    uint32_t type;       /* WKB type (1–6) */
    double  *x;          /* x coordinates */
    double  *y;          /* y coordinates */
    double  *z;          /* z coordinates (NULL if 2D) */
    uint32_t n_coords;   /* number of coordinates */
    uint32_t *offsets;   /* ring/part offsets */
    uint32_t n_offsets;  /* number of offsets */
    uint32_t *parts;     /* polygon part offsets (multi only) */
    uint32_t n_parts;    /* number of parts */
} arpt_geom;

/* Compute the bounding box of a geometry: bbox[0]=min_x, bbox[1]=min_y,
   bbox[2]=max_x, bbox[3]=max_y. Geometry must have n_coords > 0. */
void arpt_geom_bbox(const arpt_geom *g, double bbox[4]);

/* Free memory owned by an arpt_geom. */
void arpt_geom_free(arpt_geom *g);

#endif
