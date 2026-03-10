#include "tile_build.h"

#include <stdlib.h>

struct arpt_tile_builder {
    arpt_bounds bounds;
};

arpt_tile_builder *arpt_tile_builder_create(arpt_bounds bounds) {
    (void)bounds;
    return NULL;
}

bool arpt_tile_builder_add_feature(arpt_tile_builder *b,
                                   const arpt_feature *feat) {
    (void)b; (void)feat;
    return false;
}

void *arpt_tile_builder_finish(arpt_tile_builder *b, size_t *out_size) {
    (void)b;
    if (out_size) *out_size = 0;
    return NULL;
}

void arpt_tile_builder_free(arpt_tile_builder *b) {
    free(b);
}
