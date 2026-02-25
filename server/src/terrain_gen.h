#ifndef ARPENTRY_TERRAIN_GEN_H
#define ARPENTRY_TERRAIN_GEN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Generate a Brotli-compressed .arpt terrain tile for the given tile coords.
 *
 * Uses fractal noise (simplex + fBm) to produce a 32x32 elevation mesh.
 * The result is deterministic: same (level, x, y) always produces identical
 * output, and adjacent tiles share edge elevations automatically.
 *
 * @param level  Zoom level (0-21).
 * @param x      Tile column.
 * @param y      Tile row.
 * @param out    Receives malloc'd compressed buffer (caller must free).
 * @param out_size Receives compressed size in bytes.
 * @return true on success.
 */
bool arpt_generate_terrain(int level, int x, int y,
                           uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_TERRAIN_GEN_H */
