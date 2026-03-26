#include "workqueue.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

struct arpt_workqueue {
    arpt_batch *ring;       /* Ring buffer of batches */
    size_t      cap;        /* Ring capacity (max_batches) */
    size_t      head;       /* Next write position */
    size_t      tail;       /* Next read position */
    size_t      count;      /* Current number of batches */

    pthread_mutex_t mutex;
    pthread_cond_t  not_full;
    pthread_cond_t  not_empty;
    bool            closed;
};

arpt_workqueue *arpt_workqueue_create(size_t max_batches) {
    if (max_batches == 0) max_batches = 1;

    arpt_workqueue *q = calloc(1, sizeof(*q));
    if (!q) return NULL;

    q->ring = calloc(max_batches, sizeof(arpt_batch));
    if (!q->ring) { free(q); return NULL; }

    q->cap = max_batches;

    if (pthread_mutex_init(&q->mutex, NULL) != 0) {
        free(q->ring);
        free(q);
        return NULL;
    }
    if (pthread_cond_init(&q->not_full, NULL) != 0) {
        pthread_mutex_destroy(&q->mutex);
        free(q->ring);
        free(q);
        return NULL;
    }
    if (pthread_cond_init(&q->not_empty, NULL) != 0) {
        pthread_cond_destroy(&q->not_full);
        pthread_mutex_destroy(&q->mutex);
        free(q->ring);
        free(q);
        return NULL;
    }

    return q;
}

bool arpt_workqueue_push(arpt_workqueue *q, arpt_batch *batch) {
    if (!q || !batch) return false;

    pthread_mutex_lock(&q->mutex);

    while (q->count == q->cap && !q->closed) {
        pthread_cond_wait(&q->not_full, &q->mutex);
    }

    if (q->closed) {
        pthread_mutex_unlock(&q->mutex);
        return false;
    }

    q->ring[q->head] = *batch;
    batch->items = NULL;
    batch->count = 0;
    q->head = (q->head + 1) % q->cap;
    q->count++;

    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mutex);
    return true;
}

bool arpt_workqueue_pop(arpt_workqueue *q, arpt_batch *batch) {
    if (!q || !batch) return false;

    pthread_mutex_lock(&q->mutex);

    while (q->count == 0 && !q->closed) {
        pthread_cond_wait(&q->not_empty, &q->mutex);
    }

    if (q->count == 0) {
        /* Closed and empty */
        pthread_mutex_unlock(&q->mutex);
        batch->items = NULL;
        batch->count = 0;
        return false;
    }

    *batch = q->ring[q->tail];
    q->ring[q->tail].items = NULL;
    q->ring[q->tail].count = 0;
    q->tail = (q->tail + 1) % q->cap;
    q->count--;

    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->mutex);
    return true;
}

void arpt_workqueue_close(arpt_workqueue *q) {
    if (!q) return;

    pthread_mutex_lock(&q->mutex);
    q->closed = true;
    pthread_cond_broadcast(&q->not_full);
    pthread_cond_broadcast(&q->not_empty);
    pthread_mutex_unlock(&q->mutex);
}

void arpt_workqueue_free(arpt_workqueue *q) {
    if (!q) return;

    /* Free any remaining batches in the ring */
    for (size_t i = 0; i < q->count; i++) {
        size_t idx = (q->tail + i) % q->cap;
        free(q->ring[idx].items);
    }

    pthread_cond_destroy(&q->not_empty);
    pthread_cond_destroy(&q->not_full);
    pthread_mutex_destroy(&q->mutex);
    free(q->ring);
    free(q);
}
