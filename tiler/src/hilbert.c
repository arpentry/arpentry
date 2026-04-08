#include "hilbert.h"

/* Standard Hilbert curve algorithm using bit manipulation.
   Reference: Wikipedia "Hilbert curve" / John Skilling's method. */

static void rot(uint32_t n, uint32_t *x, uint32_t *y, uint32_t rx, uint32_t ry) {
    if (ry == 0) {
        if (rx == 1) {
            *x = n - 1 - *x;
            *y = n - 1 - *y;
        }
        uint32_t tmp = *x;
        *x = *y;
        *y = tmp;
    }
}

uint64_t arpt_hilbert_xy2d(int order, uint32_t x, uint32_t y) {
    uint64_t d = 0;
    for (uint32_t s = (1u << order) >> 1; s > 0; s >>= 1) {
        uint32_t rx = (x & s) ? 1 : 0;
        uint32_t ry = (y & s) ? 1 : 0;
        d += (uint64_t)s * s * ((3 * rx) ^ ry);
        rot(s << 1, &x, &y, rx, ry);
    }
    return d;
}

void arpt_hilbert_d2xy(int order, uint64_t d, uint32_t *x, uint32_t *y) {
    if (!x || !y) return;
    uint32_t rx, ry;
    *x = 0;
    *y = 0;
    for (uint32_t s = 1; s < (1u << order); s <<= 1) {
        rx = 1 & (uint32_t)(d / 2);
        ry = 1 & ((uint32_t)d ^ rx);
        rot(s, x, y, rx, ry);
        *x += s * rx;
        *y += s * ry;
        d /= 4;
    }
}

/*
 * Tile ID layout (48 bits):
 *   bits [47..42]  zoom   (6 bits, max 63)
 *   bits [41..0]   hilbert distance (42 bits, sufficient up to z=20)
 *
 * At zoom z, the grid is 2^z columns × 2^z rows.
 * Embed into a 2^z square for Hilbert indexing (order = z).
 */

uint64_t arpt_hilbert_tile_id(int z, int x, int y) {
    int order = z;
    uint64_t h = arpt_hilbert_xy2d(order, (uint32_t)x, (uint32_t)y);
    return ((uint64_t)z << 42) | (h & 0x3FFFFFFFFFFull);
}

void arpt_hilbert_tile_id_decode(uint64_t id, int *z, int *x, int *y) {
    int zz = (int)(id >> 42) & 0x3F;
    uint64_t h = id & 0x3FFFFFFFFFFull;
    uint32_t tx = 0, ty = 0;
    arpt_hilbert_d2xy(zz, h, &tx, &ty);
    if (z) *z = zz;
    if (x) *x = (int)tx;
    if (y) *y = (int)ty;
}

