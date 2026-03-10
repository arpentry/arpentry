#include "wkb.h"

#include <stdlib.h>

bool arpt_wkb_parse(const uint8_t *data, size_t size, arpt_geom *out) {
    (void)data; (void)size; (void)out;
    return false;
}

void arpt_geom_free(arpt_geom *g) {
    if (!g) return;
    free(g->x);
    free(g->y);
    free(g->z);
    free(g->offsets);
    free(g->parts);
}
