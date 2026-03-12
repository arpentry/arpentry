/* Bump allocator with block chaining and O(1) reset. */

#ifndef ARPT_ARENA_H
#define ARPT_ARENA_H

#include <stddef.h>

typedef struct arpt_arena arpt_arena;

/* Create an arena with the given initial block size. */
arpt_arena *arpt_arena_create(size_t block_size);

/* Allocate n bytes (8-byte aligned). Returns NULL on failure. */
void *arpt_arena_alloc(arpt_arena *a, size_t n);

/* Reset all allocations without freeing memory. O(1). */
void arpt_arena_reset(arpt_arena *a);

/* Free the arena and all blocks. */
void arpt_arena_free(arpt_arena *a);

#endif
