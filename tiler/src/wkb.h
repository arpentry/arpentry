/* WKB geometry parser. */

#ifndef ARPT_WKB_H
#define ARPT_WKB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

/* Parse a WKB blob into an arpt_geom. Returns false on error. */
bool arpt_wkb_parse(const uint8_t *data, size_t size, arpt_geom *out);

/* Free memory owned by an arpt_geom. */
void arpt_geom_free(arpt_geom *g);

#endif
