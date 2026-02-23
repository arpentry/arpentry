#include "tile_path.h"
#include <stdio.h>
#include <string.h>

bool arpt_parse_tile_path(const char *uri, int *level, int *x, int *y) {
    if (!uri || !level || !x || !y) return false;

    int l, tx, ty;
    char ext[8] = {0};
    int n = sscanf(uri, "/%d/%d/%d%7s", &l, &tx, &ty, ext);
    if (n != 4) return false;
    if (strcmp(ext, ".arpt") != 0) return false;

    /* Level must be in [0, 21] */
    if (l < 0 || l > 21) return false;

    /* x in [0, 2^(level+1) - 1], y in [0, 2^level - 1] */
    int max_x = (1 << (l + 1)) - 1;
    int max_y = (1 << l) - 1;
    if (tx < 0 || tx > max_x) return false;
    if (ty < 0 || ty > max_y) return false;

    *level = l;
    *x = tx;
    *y = ty;
    return true;
}
