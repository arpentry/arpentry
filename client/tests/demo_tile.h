#ifndef ARPENTRY_DEMO_TILE_H
#define ARPENTRY_DEMO_TILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Build a demo tile with all geometry types, property types, and mesh materials.
 * Caller frees *out. */
bool arpt_build_demo_tile(uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_DEMO_TILE_H */
