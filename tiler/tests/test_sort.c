#include "unity.h"
#include "sort.h"
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static void test_create_free(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024);
    TEST_ASSERT_NOT_NULL(s);
    arpt_sorter_free(s);
}

static void test_free_null(void) {
    arpt_sorter_free(NULL);
}

static void test_empty_sorter(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_TRUE(arpt_sorter_finish(s));

    uint64_t key;
    const void *data;
    size_t size;
    TEST_ASSERT_FALSE(arpt_sorter_next(s, &key, &data, &size));
    arpt_sorter_free(s);
}

static void test_single_record(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024 * 1024);
    TEST_ASSERT_NOT_NULL(s);

    const char *payload = "hello";
    TEST_ASSERT_TRUE(arpt_sorter_add(s, 42, payload, 5));
    TEST_ASSERT_TRUE(arpt_sorter_finish(s));

    uint64_t key;
    const void *data;
    size_t size;
    TEST_ASSERT_TRUE(arpt_sorter_next(s, &key, &data, &size));
    TEST_ASSERT_EQUAL_UINT64(42, key);
    TEST_ASSERT_EQUAL_size_t(5, size);
    TEST_ASSERT_EQUAL_MEMORY("hello", data, 5);

    TEST_ASSERT_FALSE(arpt_sorter_next(s, &key, &data, &size));
    arpt_sorter_free(s);
}

static void test_sorted_output(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024 * 1024);
    TEST_ASSERT_NOT_NULL(s);

    /* Add in reverse order */
    uint64_t keys[] = {100, 50, 200, 10, 150};
    for (int i = 0; i < 5; i++) {
        uint8_t val = (uint8_t)i;
        TEST_ASSERT_TRUE(arpt_sorter_add(s, keys[i], &val, 1));
    }
    TEST_ASSERT_TRUE(arpt_sorter_finish(s));

    /* Should come out sorted */
    uint64_t expected[] = {10, 50, 100, 150, 200};
    for (int i = 0; i < 5; i++) {
        uint64_t key;
        const void *data;
        size_t size;
        TEST_ASSERT_TRUE(arpt_sorter_next(s, &key, &data, &size));
        TEST_ASSERT_EQUAL_UINT64(expected[i], key);
    }

    uint64_t key;
    TEST_ASSERT_FALSE(arpt_sorter_next(s, &key, NULL, NULL));
    arpt_sorter_free(s);
}

static void test_zero_length_data(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024 * 1024);
    TEST_ASSERT_NOT_NULL(s);

    TEST_ASSERT_TRUE(arpt_sorter_add(s, 1, NULL, 0));
    TEST_ASSERT_TRUE(arpt_sorter_add(s, 2, NULL, 0));
    TEST_ASSERT_TRUE(arpt_sorter_finish(s));

    uint64_t key;
    const void *data;
    size_t size;
    TEST_ASSERT_TRUE(arpt_sorter_next(s, &key, &data, &size));
    TEST_ASSERT_EQUAL_UINT64(1, key);
    TEST_ASSERT_EQUAL_size_t(0, size);
    TEST_ASSERT_TRUE(arpt_sorter_next(s, &key, &data, &size));
    TEST_ASSERT_EQUAL_UINT64(2, key);
    arpt_sorter_free(s);
}

static void test_spill_to_disk(void) {
    /* Tiny budget forces spilling after a few records */
    arpt_sorter *s = arpt_sorter_create("/tmp", 64);
    TEST_ASSERT_NOT_NULL(s);

    /* Each record is 12 bytes header + 4 bytes data = 16 bytes.
       Budget of 64 means ~4 records per run. */
    uint64_t keys[] = {90, 30, 70, 10, 50, 80, 20, 60, 40, 100};
    for (int i = 0; i < 10; i++) {
        uint32_t val = (uint32_t)keys[i];
        TEST_ASSERT_TRUE(arpt_sorter_add(s, keys[i], &val, sizeof(val)));
    }
    TEST_ASSERT_TRUE(arpt_sorter_finish(s));

    /* Verify globally sorted output */
    uint64_t prev = 0;
    int count = 0;
    uint64_t key;
    const void *data;
    size_t size;
    while (arpt_sorter_next(s, &key, &data, &size)) {
        TEST_ASSERT_TRUE(key >= prev);
        prev = key;
        count++;
    }
    TEST_ASSERT_EQUAL_INT(10, count);
    arpt_sorter_free(s);
}

static void test_duplicate_keys(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024 * 1024);
    TEST_ASSERT_NOT_NULL(s);

    for (int i = 0; i < 5; i++) {
        uint8_t val = (uint8_t)i;
        TEST_ASSERT_TRUE(arpt_sorter_add(s, 42, &val, 1));
    }
    TEST_ASSERT_TRUE(arpt_sorter_finish(s));

    int count = 0;
    uint64_t key;
    while (arpt_sorter_next(s, &key, NULL, NULL)) {
        TEST_ASSERT_EQUAL_UINT64(42, key);
        count++;
    }
    TEST_ASSERT_EQUAL_INT(5, count);
    arpt_sorter_free(s);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_create_free);
    RUN_TEST(test_free_null);
    RUN_TEST(test_empty_sorter);
    RUN_TEST(test_single_record);
    RUN_TEST(test_sorted_output);
    RUN_TEST(test_zero_length_data);
    RUN_TEST(test_spill_to_disk);
    RUN_TEST(test_duplicate_keys);
    return UNITY_END();
}
