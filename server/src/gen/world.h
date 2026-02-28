#ifndef ARPENTRY_GEN_WORLD_H
#define ARPENTRY_GEN_WORLD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Generate a compressed .arpt terrain tile using fractal noise (simplex + fBm).
 * Deterministic: same coords always produce identical output. Caller frees
 * *out. */
bool arpt_generate_terrain(int level, int x, int y, uint8_t **out,
                           size_t *out_size);

#endif /* ARPENTRY_GEN_WORLD_H */
