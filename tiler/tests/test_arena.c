#include "unity.h"
#include "arena.h"

#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* ---- Creation / destruction ---- */

static void test_create_and_free(void) {
    arpt_arena *a = arpt_arena_create(1024);
    TEST_ASSERT_NOT_NULL(a);
    arpt_arena_free(a);
}

static void test_create_zero_block_size(void) {
    /* block_size == 0 should use a default, not crash */
    arpt_arena *a = arpt_arena_create(0);
    TEST_ASSERT_NOT_NULL(a);
    arpt_arena_free(a);
}

static void test_free_null(void) {
    /* Should not crash */
    arpt_arena_free(NULL);
}

/* ---- Allocation ---- */

static void test_alloc_basic(void) {
    arpt_arena *a = arpt_arena_create(1024);
    TEST_ASSERT_NOT_NULL(a);

    void *p = arpt_arena_alloc(a, 64);
    TEST_ASSERT_NOT_NULL(p);

    /* Should be writable */
    memset(p, 0xAB, 64);

    arpt_arena_free(a);
}

static void test_alloc_zero_returns_null(void) {
    arpt_arena *a = arpt_arena_create(1024);
    TEST_ASSERT_NOT_NULL(a);

    void *p = arpt_arena_alloc(a, 0);
    TEST_ASSERT_NULL(p);

    arpt_arena_free(a);
}

static void test_alloc_null_arena(void) {
    void *p = arpt_arena_alloc(NULL, 64);
    TEST_ASSERT_NULL(p);
}

static void test_alloc_alignment(void) {
    arpt_arena *a = arpt_arena_create(1024);
    TEST_ASSERT_NOT_NULL(a);

    /* Allocate odd sizes and check 8-byte alignment */
    for (int i = 0; i < 10; i++) {
        void *p = arpt_arena_alloc(a, 3 + (size_t)i);
        TEST_ASSERT_NOT_NULL(p);
        TEST_ASSERT_EQUAL_UINT64(0, (uintptr_t)p % 8);
    }

    arpt_arena_free(a);
}

static void test_alloc_multiple_no_overlap(void) {
    arpt_arena *a = arpt_arena_create(1024);
    TEST_ASSERT_NOT_NULL(a);

    void *p1 = arpt_arena_alloc(a, 100);
    void *p2 = arpt_arena_alloc(a, 100);
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_NOT_NULL(p2);

    /* Pointers must not overlap */
    uintptr_t a1 = (uintptr_t)p1;
    uintptr_t a2 = (uintptr_t)p2;
    TEST_ASSERT_TRUE(a2 >= a1 + 100 || a1 >= a2 + 100);

    arpt_arena_free(a);
}

static void test_alloc_exceeds_block(void) {
    /* Small block size forces a new block allocation */
    arpt_arena *a = arpt_arena_create(64);
    TEST_ASSERT_NOT_NULL(a);

    void *p1 = arpt_arena_alloc(a, 48);
    TEST_ASSERT_NOT_NULL(p1);

    /* This should spill into a new block */
    void *p2 = arpt_arena_alloc(a, 48);
    TEST_ASSERT_NOT_NULL(p2);

    memset(p1, 0x11, 48);
    memset(p2, 0x22, 48);

    /* Verify no corruption */
    uint8_t *b1 = (uint8_t *)p1;
    uint8_t *b2 = (uint8_t *)p2;
    for (int i = 0; i < 48; i++) {
        TEST_ASSERT_EQUAL_UINT8(0x11, b1[i]);
        TEST_ASSERT_EQUAL_UINT8(0x22, b2[i]);
    }

    arpt_arena_free(a);
}

static void test_alloc_larger_than_block(void) {
    /* Allocation larger than block_size should still succeed */
    arpt_arena *a = arpt_arena_create(64);
    TEST_ASSERT_NOT_NULL(a);

    void *p = arpt_arena_alloc(a, 256);
    TEST_ASSERT_NOT_NULL(p);
    memset(p, 0xCC, 256);

    arpt_arena_free(a);
}

/* ---- Reset ---- */

static void test_reset_basic(void) {
    arpt_arena *a = arpt_arena_create(1024);
    TEST_ASSERT_NOT_NULL(a);

    void *p1 = arpt_arena_alloc(a, 100);
    TEST_ASSERT_NOT_NULL(p1);

    arpt_arena_reset(a);

    /* After reset, allocations reuse the same block memory */
    void *p2 = arpt_arena_alloc(a, 100);
    TEST_ASSERT_NOT_NULL(p2);

    /* p2 should start at the same position as p1 since we reset */
    TEST_ASSERT_EQUAL_PTR(p1, p2);

    arpt_arena_free(a);
}

static void test_reset_null(void) {
    /* Should not crash */
    arpt_arena_reset(NULL);
}

static void test_reset_reuses_blocks(void) {
    /* Force multiple blocks, reset, then allocate again */
    arpt_arena *a = arpt_arena_create(64);
    TEST_ASSERT_NOT_NULL(a);

    arpt_arena_alloc(a, 48);
    arpt_arena_alloc(a, 48); /* spills to block 2 */
    arpt_arena_alloc(a, 48); /* spills to block 3 */

    arpt_arena_reset(a);

    /* Should be able to allocate the same amounts again */
    void *p1 = arpt_arena_alloc(a, 48);
    void *p2 = arpt_arena_alloc(a, 48);
    void *p3 = arpt_arena_alloc(a, 48);
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_NOT_NULL(p2);
    TEST_ASSERT_NOT_NULL(p3);

    arpt_arena_free(a);
}

/* ---- Stress ---- */

static void test_many_small_allocations(void) {
    arpt_arena *a = arpt_arena_create(256);
    TEST_ASSERT_NOT_NULL(a);

    for (int i = 0; i < 1000; i++) {
        void *p = arpt_arena_alloc(a, 8);
        TEST_ASSERT_NOT_NULL(p);
    }

    arpt_arena_free(a);
}

static void test_reset_and_reuse_cycle(void) {
    arpt_arena *a = arpt_arena_create(256);
    TEST_ASSERT_NOT_NULL(a);

    for (int cycle = 0; cycle < 100; cycle++) {
        for (int i = 0; i < 50; i++) {
            void *p = arpt_arena_alloc(a, 16);
            TEST_ASSERT_NOT_NULL(p);
        }
        arpt_arena_reset(a);
    }

    arpt_arena_free(a);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_create_and_free);
    RUN_TEST(test_create_zero_block_size);
    RUN_TEST(test_free_null);
    RUN_TEST(test_alloc_basic);
    RUN_TEST(test_alloc_zero_returns_null);
    RUN_TEST(test_alloc_null_arena);
    RUN_TEST(test_alloc_alignment);
    RUN_TEST(test_alloc_multiple_no_overlap);
    RUN_TEST(test_alloc_exceeds_block);
    RUN_TEST(test_alloc_larger_than_block);
    RUN_TEST(test_reset_basic);
    RUN_TEST(test_reset_null);
    RUN_TEST(test_reset_reuses_blocks);
    RUN_TEST(test_many_small_allocations);
    RUN_TEST(test_reset_and_reuse_cycle);
    return UNITY_END();
}
