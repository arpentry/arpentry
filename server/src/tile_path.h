#ifndef ARPENTRY_TILE_PATH_H
#define ARPENTRY_TILE_PATH_H

#include <stdbool.h>
#include <stdint.h>

/* Parse "/{level}/{x}/{y}.arpt" URI. Validates ranges. */
bool arpt_parse_tile_path(const char *uri, int *level, int *x, int *y);

#endif /* ARPENTRY_TILE_PATH_H */
