#include "unity.h"
#include "workqueue.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static void test_create_free(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);
    TEST_ASSERT_NOT_NULL(q);
    arpt_workqueue_free(q);
}

static void test_free_null(void) {
    arpt_workqueue_free(NULL);
}

static void test_push_pop_single(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);
    TEST_ASSERT_NOT_NULL(q);

    int *items = malloc(3 * sizeof(int));
    items[0] = 10; items[1] = 20; items[2] = 30;
    arpt_batch batch = { .items = items, .count = 3 };

    TEST_ASSERT_TRUE(arpt_workqueue_push(q, &batch));
    TEST_ASSERT_NULL(batch.items);  /* Ownership transferred */

    arpt_batch out = {0};
    TEST_ASSERT_TRUE(arpt_workqueue_pop(q, &out));
    TEST_ASSERT_EQUAL_size_t(3, out.count);
    int *vals = (int *)out.items;
    TEST_ASSERT_EQUAL_INT(10, vals[0]);
    TEST_ASSERT_EQUAL_INT(20, vals[1]);
    TEST_ASSERT_EQUAL_INT(30, vals[2]);
    free(out.items);

    arpt_workqueue_close(q);
    arpt_workqueue_free(q);
}

static void test_close_empty_returns_false(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);
    arpt_workqueue_close(q);

    arpt_batch out = {0};
    TEST_ASSERT_FALSE(arpt_workqueue_pop(q, &out));
    TEST_ASSERT_NULL(out.items);

    arpt_workqueue_free(q);
}

static void test_push_after_close_returns_false(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);
    arpt_workqueue_close(q);

    int *items = malloc(sizeof(int));
    items[0] = 1;
    arpt_batch batch = { .items = items, .count = 1 };
    TEST_ASSERT_FALSE(arpt_workqueue_push(q, &batch));
    free(batch.items);  /* Still owned since push failed */

    arpt_workqueue_free(q);
}

static void test_drain_after_close(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);

    for (int i = 0; i < 3; i++) {
        int *item = malloc(sizeof(int));
        *item = i;
        arpt_batch b = { .items = item, .count = 1 };
        TEST_ASSERT_TRUE(arpt_workqueue_push(q, &b));
    }

    arpt_workqueue_close(q);

    /* Should still be able to pop existing items */
    for (int i = 0; i < 3; i++) {
        arpt_batch out = {0};
        TEST_ASSERT_TRUE(arpt_workqueue_pop(q, &out));
        TEST_ASSERT_EQUAL_INT(i, *(int *)out.items);
        free(out.items);
    }

    /* Now empty and closed */
    arpt_batch out = {0};
    TEST_ASSERT_FALSE(arpt_workqueue_pop(q, &out));

    arpt_workqueue_free(q);
}

/* ---- Concurrent tests ---- */

typedef struct {
    arpt_workqueue *q;
    int             n_batches;
    int             batch_size;
} producer_args;

static void *producer_fn(void *arg) {
    producer_args *a = (producer_args *)arg;
    for (int i = 0; i < a->n_batches; i++) {
        int *items = malloc((size_t)a->batch_size * sizeof(int));
        for (int j = 0; j < a->batch_size; j++) {
            items[j] = i * a->batch_size + j;
        }
        arpt_batch batch = { .items = items, .count = (size_t)a->batch_size };
        if (!arpt_workqueue_push(a->q, &batch)) {
            free(batch.items);
            break;
        }
    }
    return NULL;
}

typedef struct {
    arpt_workqueue *q;
    int             total_items;
} consumer_args;

static void *consumer_fn(void *arg) {
    consumer_args *a = (consumer_args *)arg;
    int count = 0;
    arpt_batch batch;
    while (arpt_workqueue_pop(a->q, &batch)) {
        count += (int)batch.count;
        free(batch.items);
    }
    int *result = malloc(sizeof(int));
    *result = count;
    return result;
}

static void test_concurrent_producer_consumer(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);

    int n_batches = 100;
    int batch_size = 64;
    producer_args pargs = { .q = q, .n_batches = n_batches, .batch_size = batch_size };
    consumer_args cargs = { .q = q, .total_items = n_batches * batch_size };

    pthread_t prod, cons;
    pthread_create(&prod, NULL, producer_fn, &pargs);
    pthread_create(&cons, NULL, consumer_fn, &cargs);

    pthread_join(prod, NULL);
    arpt_workqueue_close(q);

    void *result;
    pthread_join(cons, &result);
    int consumed = *(int *)result;
    free(result);

    TEST_ASSERT_EQUAL_INT(n_batches * batch_size, consumed);
    arpt_workqueue_free(q);
}

static void test_multiple_producers_single_consumer(void) {
    arpt_workqueue *q = arpt_workqueue_create(4);
    int n_producers = 4;
    int n_batches = 50;
    int batch_size = 32;

    producer_args pargs = { .q = q, .n_batches = n_batches, .batch_size = batch_size };
    consumer_args cargs = { .q = q, .total_items = 0 };

    pthread_t prods[4], cons;
    for (int i = 0; i < n_producers; i++) {
        pthread_create(&prods[i], NULL, producer_fn, &pargs);
    }
    pthread_create(&cons, NULL, consumer_fn, &cargs);

    for (int i = 0; i < n_producers; i++) {
        pthread_join(prods[i], NULL);
    }
    arpt_workqueue_close(q);

    void *result;
    pthread_join(cons, &result);
    int consumed = *(int *)result;
    free(result);

    TEST_ASSERT_EQUAL_INT(n_producers * n_batches * batch_size, consumed);
    arpt_workqueue_free(q);
}

static void test_backpressure(void) {
    /* Queue capacity 2: producer of 10 batches must block */
    arpt_workqueue *q = arpt_workqueue_create(2);

    producer_args pargs = { .q = q, .n_batches = 10, .batch_size = 1 };
    consumer_args cargs = { .q = q, .total_items = 10 };

    pthread_t prod, cons;
    pthread_create(&prod, NULL, producer_fn, &pargs);
    pthread_create(&cons, NULL, consumer_fn, &cargs);

    pthread_join(prod, NULL);
    arpt_workqueue_close(q);

    void *result;
    pthread_join(cons, &result);
    int consumed = *(int *)result;
    free(result);

    TEST_ASSERT_EQUAL_INT(10, consumed);
    arpt_workqueue_free(q);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_create_free);
    RUN_TEST(test_free_null);
    RUN_TEST(test_push_pop_single);
    RUN_TEST(test_close_empty_returns_false);
    RUN_TEST(test_push_after_close_returns_false);
    RUN_TEST(test_drain_after_close);
    RUN_TEST(test_concurrent_producer_consumer);
    RUN_TEST(test_multiple_producers_single_consumer);
    RUN_TEST(test_backpressure);
    return UNITY_END();
}
