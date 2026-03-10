#include "sort.h"

#include <stdlib.h>

struct arpt_sorter {
    char *tmp_dir;
    size_t mem_budget;
};

arpt_sorter *arpt_sorter_create(const char *tmp_dir, size_t mem_budget) {
    (void)tmp_dir; (void)mem_budget;
    return NULL;
}

bool arpt_sorter_add(arpt_sorter *s, uint64_t key,
                     const void *data, size_t size) {
    (void)s; (void)key; (void)data; (void)size;
    return false;
}

bool arpt_sorter_finish(arpt_sorter *s) {
    (void)s;
    return false;
}

bool arpt_sorter_next(arpt_sorter *s, uint64_t *key,
                      const void **data, size_t *size) {
    (void)s; (void)key; (void)data; (void)size;
    return false;
}

void arpt_sorter_free(arpt_sorter *s) {
    if (!s) return;
    free(s->tmp_dir);
    free(s);
}
