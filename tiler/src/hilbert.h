/* Hilbert curve mapping for tile ordering. */

#ifndef ARPT_HILBERT_H
#define ARPT_HILBERT_H

#include <stdint.h>

/* Convert (x, y) to Hilbert distance on a 2^order square grid. */
uint64_t arpt_hilbert_xy2d(int order, uint32_t x, uint32_t y);

/* Convert Hilbert distance to (x, y) on a 2^order square grid. */
void arpt_hilbert_d2xy(int order, uint64_t d, uint32_t *x, uint32_t *y);

/* Encode a tile (z, x, y) into a zoom-prefixed Hilbert tile ID. */
uint64_t arpt_hilbert_tile_id(int z, int x, int y);

/* Decode a Hilbert tile ID back into (z, x, y). */
void arpt_hilbert_tile_id_decode(uint64_t id, int *z, int *x, int *y);

#endif
