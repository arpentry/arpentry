#include "simplify.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Perpendicular distance from point (px,py) to line (ax,ay)-(bx,by). */
static double perp_dist(double px, double py,
                        double ax, double ay, double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;
    double len_sq = dx * dx + dy * dy;
    if (len_sq == 0.0) {
        dx = px - ax;
        dy = py - ay;
        return sqrt(dx * dx + dy * dy);
    }
    double area2 = fabs(dy * px - dx * py + bx * ay - by * ax);
    return area2 / sqrt(len_sq);
}

/* Iterative Douglas-Peucker using an explicit stack. */
uint32_t arpt_simplify(double *x, double *y, uint32_t count, double tolerance) {
    if (count <= 2) return count;
    if (tolerance <= 0.0) return count;

    /* keep[i] marks whether vertex i survives */
    bool *keep = calloc(count, sizeof(*keep));
    if (!keep) return count;

    keep[0] = true;
    keep[count - 1] = true;

    /* Stack of (start, end) index pairs */
    uint32_t *stack = malloc(count * 2 * sizeof(*stack));
    if (!stack) { free(keep); return count; }

    uint32_t sp = 0;
    stack[sp++] = 0;
    stack[sp++] = count - 1;

    while (sp >= 2) {
        uint32_t end   = stack[--sp];
        uint32_t start = stack[--sp];

        double max_dist = 0.0;
        uint32_t max_idx = start;
        for (uint32_t i = start + 1; i < end; i++) {
            double d = perp_dist(x[i], y[i],
                                 x[start], y[start], x[end], y[end]);
            if (d > max_dist) {
                max_dist = d;
                max_idx = i;
            }
        }

        if (max_dist > tolerance) {
            keep[max_idx] = true;
            stack[sp++] = start;
            stack[sp++] = max_idx;
            stack[sp++] = max_idx;
            stack[sp++] = end;
        }
    }

    /* Compact in-place */
    uint32_t out = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (keep[i]) {
            x[out] = x[i];
            y[out] = y[i];
            out++;
        }
    }

    free(stack);
    free(keep);
    return out;
}
