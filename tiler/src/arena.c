/* Bump allocator with block chaining and O(1) reset. */

#include "arena.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ARENA_ALIGN 8

typedef struct arena_block {
    struct arena_block *next;
    size_t cap;
    size_t used;
    /* data follows (flexible array member) */
    uint8_t data[];
} arena_block;

struct arpt_arena {
    arena_block *head;      /* current (active) block */
    arena_block *first;     /* first block (for reset) */
    size_t block_size;      /* default block capacity */
};

static arena_block *block_create(size_t cap) {
    arena_block *b = malloc(sizeof(arena_block) + cap);
    if (!b) return NULL;
    b->next = NULL;
    b->cap = cap;
    b->used = 0;
    return b;
}

arpt_arena *arpt_arena_create(size_t block_size) {
    if (block_size == 0) block_size = 64 * 1024;
    arpt_arena *a = malloc(sizeof(*a));
    if (!a) return NULL;

    arena_block *b = block_create(block_size);
    if (!b) { free(a); return NULL; }

    a->head = b;
    a->first = b;
    a->block_size = block_size;
    return a;
}

void *arpt_arena_alloc(arpt_arena *a, size_t n) {
    if (!a || n == 0) return NULL;

    /* Align up */
    n = (n + ARENA_ALIGN - 1) & ~(size_t)(ARENA_ALIGN - 1);

    /* Try current block */
    arena_block *b = a->head;
    if (b->used + n <= b->cap) {
        void *ptr = b->data + b->used;
        b->used += n;
        return ptr;
    }

    /* Try next block (from a previous reset cycle) */
    if (b->next && b->next->cap >= n) {
        a->head = b->next;
        a->head->used = n;
        return a->head->data;
    }

    /* Allocate a new block */
    size_t cap = a->block_size;
    if (n > cap) cap = n;
    arena_block *nb = block_create(cap);
    if (!nb) return NULL;

    nb->next = b->next;
    b->next = nb;
    a->head = nb;
    nb->used = n;
    return nb->data;
}

void arpt_arena_reset(arpt_arena *a) {
    if (!a) return;
    /* Reset used counters on all blocks, rewind to first */
    for (arena_block *b = a->first; b; b = b->next)
        b->used = 0;
    a->head = a->first;
}

void arpt_arena_free(arpt_arena *a) {
    if (!a) return;
    arena_block *b = a->first;
    while (b) {
        arena_block *next = b->next;
        free(b);
        b = next;
    }
    free(a);
}
