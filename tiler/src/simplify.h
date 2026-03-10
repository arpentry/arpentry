/* Douglas-Peucker line simplification. */

#ifndef ARPT_SIMPLIFY_H
#define ARPT_SIMPLIFY_H

#include <stdint.h>

/* Simplify a polyline in-place. Returns the new vertex count. */
uint32_t arpt_simplify(double *x, double *y, uint32_t count, double tolerance);

#endif
