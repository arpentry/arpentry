#ifndef ARPENTRY_TILE_PATH_H
#define ARPENTRY_TILE_PATH_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Parse a tile URI of the form "/{level}/{x}/{y}.arpt".
 *
 * Validates that level is in [0, 21], x in [0, 2^(level+1)-1],
 * and y in [0, 2^level - 1].
 *
 * @param uri   The request URI to parse.
 * @param level Receives the zoom level.
 * @param x     Receives the tile column.
 * @param y     Receives the tile row.
 * @return true if the URI matched and all values are in range.
 */
bool arpt_parse_tile_path(const char *uri, int *level, int *x, int *y);

#endif /* ARPENTRY_TILE_PATH_H */
