#ifndef ARPENTRY_DEMO_TILE_H
#define ARPENTRY_DEMO_TILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Build a demonstration tile showcasing all format capabilities.
 *
 * Creates a Brotli-compressed .arpt buffer with 4 layers (one per geometry
 * type), all property value types, and mesh parts with inline materials.
 *
 * @param out      Receives malloc'd compressed buffer (caller must free).
 * @param out_size Receives compressed size in bytes.
 * @return true on success.
 */
bool arpt_build_demo_tile(uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_DEMO_TILE_H */
