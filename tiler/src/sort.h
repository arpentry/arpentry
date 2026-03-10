/* External merge sort with configurable memory budget. */

#ifndef ARPT_SORT_H
#define ARPT_SORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct arpt_sorter arpt_sorter;

/* Create a sorter. tmp_dir is used for spill files. */
arpt_sorter *arpt_sorter_create(const char *tmp_dir, size_t mem_budget);

/* Add a record (key + variable-length data). */
bool arpt_sorter_add(arpt_sorter *s, uint64_t key,
                     const void *data, size_t size);

/* Finalize: merge all runs. Must be called before iterating. */
bool arpt_sorter_finish(arpt_sorter *s);

/* Read the next sorted record. Returns false when exhausted. */
bool arpt_sorter_next(arpt_sorter *s, uint64_t *key,
                      const void **data, size_t *size);

/* Free the sorter and all temporary files. */
void arpt_sorter_free(arpt_sorter *s);

#endif
