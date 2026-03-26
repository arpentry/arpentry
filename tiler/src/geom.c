/* Core geometry operations. */

#include "geom.h"

#include <stdlib.h>

void arpt_geom_bbox(const arpt_geom *g, double bbox[4]) {
    if (!g || g->n_coords == 0) {
        bbox[0] = bbox[1] = bbox[2] = bbox[3] = 0.0;
        return;
    }
    bbox[0] = bbox[2] = g->x[0];
    bbox[1] = bbox[3] = g->y[0];
    for (uint32_t i = 1; i < g->n_coords; i++) {
        if (g->x[i] < bbox[0]) bbox[0] = g->x[i];
        if (g->x[i] > bbox[2]) bbox[2] = g->x[i];
        if (g->y[i] < bbox[1]) bbox[1] = g->y[i];
        if (g->y[i] > bbox[3]) bbox[3] = g->y[i];
    }
}

void arpt_geom_free(arpt_geom *g) {
    if (!g) return;
    free(g->x);
    free(g->y);
    free(g->z);
    free(g->offsets);
    free(g->parts);
}
