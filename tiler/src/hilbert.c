#include "hilbert.h"

uint64_t arpt_hilbert_xy2d(int order, uint32_t x, uint32_t y) {
    (void)order; (void)x; (void)y;
    return 0;
}

void arpt_hilbert_d2xy(int order, uint64_t d, uint32_t *x, uint32_t *y) {
    (void)order; (void)d;
    if (x) *x = 0;
    if (y) *y = 0;
}

uint64_t arpt_hilbert_tile_id(int z, int x, int y) {
    (void)z; (void)x; (void)y;
    return 0;
}

void arpt_hilbert_tile_id_decode(uint64_t id, int *z, int *x, int *y) {
    (void)id;
    if (z) *z = 0;
    if (x) *x = 0;
    if (y) *y = 0;
}
