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

/* Absolute area of a closed ring via the shoelace formula.
 * The ring must have first == last (closing vertex). */
double arpt_ring_area(const double *x, const double *y, uint32_t count);

/* Total length of an open polyline (sum of segment lengths). */
double arpt_line_length(const double *x, const double *y, uint32_t count);

/* Create a simplified copy of a geometry.  Applies Douglas-Peucker
 * with the given tolerance, then drops polygon rings with absolute
 * area < min_area and line segments with length < min_length.
 * Pass 0.0 for min_area / min_length to disable filtering.
 * Returns false if the geometry degenerates completely.  Caller must
 * free the returned geometry with arpt_geom_free(). */
bool arpt_simplify_geom(const arpt_geom *in, double tolerance,
                         double min_area, double min_length,
                         arpt_geom *out);

#endif
