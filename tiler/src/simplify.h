/* Douglas-Peucker line simplification. */

#ifndef ARPT_SIMPLIFY_H
#define ARPT_SIMPLIFY_H

#include "geom.h"

#include <stdbool.h>
#include <stdint.h>

/* Simplify an open polyline in-place. Returns the new vertex count. */
uint32_t arpt_simplify(double *x, double *y, uint32_t count, double tolerance);

/* Simplify a closed polygon ring in-place. Handles the closing
 * duplicate vertex (first == last) correctly by splitting the ring
 * at a pivot and running DP on each arc. Returns the new vertex
 * count (including the closing vertex). */
uint32_t arpt_simplify_ring(double *x, double *y, uint32_t count,
                             double tolerance);

/* Create a simplified copy of a geometry. Returns false if the
 * geometry degenerates (all rings too small). Caller must free
 * the returned geometry with arpt_geom_free(). */
bool arpt_simplify_geom(const arpt_geom *in, double tolerance,
                         arpt_geom *out);

#endif
