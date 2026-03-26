/* Bounded, batch-oriented producer-consumer queue. */

#ifndef ARPT_WORKQUEUE_H
#define ARPT_WORKQUEUE_H

#include <stdbool.h>
#include <stddef.h>

typedef struct arpt_workqueue arpt_workqueue;

/* A batch of items transferred through the queue.
   The queue takes ownership of `items` on push;
   the consumer owns `items` after pop and must free it. */
typedef struct {
    void   *items;      /* Opaque item array */
    size_t  count;      /* Number of items */
} arpt_batch;

/* Create a bounded queue. max_batches limits in-flight batches
   (producers block when full). Returns NULL on failure. */
arpt_workqueue *arpt_workqueue_create(size_t max_batches);

/* Push a batch. Blocks if the queue is full. Takes ownership of
   batch->items (sets batch->items to NULL after push).
   Returns false if the queue has been closed. */
bool arpt_workqueue_push(arpt_workqueue *q, arpt_batch *batch);

/* Pop a batch. Blocks if the queue is empty. Returns false when
   the queue is closed and empty (consumer should exit).
   On success, caller owns batch->items and must free it. */
bool arpt_workqueue_pop(arpt_workqueue *q, arpt_batch *batch);

/* Signal that no more batches will be pushed. Wakes all blocked
   consumers. Safe to call from any thread; idempotent. */
void arpt_workqueue_close(arpt_workqueue *q);

/* Free the queue. All threads must have joined first. */
void arpt_workqueue_free(arpt_workqueue *q);

#endif
