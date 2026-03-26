#include "unity.h"
#include "archive.h"
#include <stdio.h>
#include <string.h>

static const char *TEST_PATH = "/tmp/test_arpa_archive.arpa";

static arpt_archive_config test_config(void) {
    return (arpt_archive_config){
        .path = TEST_PATH,
        .min_zoom = 0,
        .max_zoom = 4,
        .bounds = {-180.0, -85.0, 180.0, 85.0},
    };
}

void setUp(void) {}
void tearDown(void) {
    remove(TEST_PATH);
}

static void test_writer_create_free(void) {
    arpt_archive_config cfg = test_config();
    arpt_archive_writer *w = arpt_archive_writer_create(&cfg);
    TEST_ASSERT_NOT_NULL(w);
    arpt_archive_writer_free(w);
}

static void test_empty_archive(void) {
    arpt_archive_config cfg = test_config();
    arpt_archive_writer *w = arpt_archive_writer_create(&cfg);
    TEST_ASSERT_NOT_NULL(w);
    TEST_ASSERT_TRUE(arpt_archive_writer_finish(w));
    arpt_archive_writer_free(w);

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_EQUAL_UINT64(0, arpt_archive_reader_tile_count(r));
    arpt_archive_reader_close(r);
}

static void test_single_tile(void) {
    arpt_archive_config cfg = test_config();
    arpt_archive_writer *w = arpt_archive_writer_create(&cfg);
    TEST_ASSERT_NOT_NULL(w);

    const char *blob = "tile-data-z0x0y0";
    TEST_ASSERT_TRUE(arpt_archive_writer_add_tile(w, 0, 0, 0,
                                                   blob, strlen(blob)));
    TEST_ASSERT_TRUE(arpt_archive_writer_finish(w));
    arpt_archive_writer_free(w);

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_EQUAL_UINT64(1, arpt_archive_reader_tile_count(r));

    size_t size;
    const void *data = arpt_archive_reader_get_tile(r, 0, 0, 0, &size);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL_size_t(strlen(blob), size);
    TEST_ASSERT_EQUAL_MEMORY(blob, data, size);

    arpt_archive_reader_close(r);
}

static void test_multiple_tiles(void) {
    arpt_archive_config cfg = {
        .path = TEST_PATH,
        .min_zoom = 0,
        .max_zoom = 2,
        .bounds = {-10.0, 40.0, 20.0, 55.0},
    };
    arpt_archive_writer *w = arpt_archive_writer_create(&cfg);
    TEST_ASSERT_NOT_NULL(w);

    /* Add tiles at various zoom levels */
    const char *blob0 = "tile-z0";
    const char *blob1a = "tile-z1-0-0";
    const char *blob1b = "tile-z1-1-0";
    const char *blob2 = "tile-z2-2-1";

    TEST_ASSERT_TRUE(arpt_archive_writer_add_tile(w, 0, 0, 0,
                                                   blob0, strlen(blob0)));
    TEST_ASSERT_TRUE(arpt_archive_writer_add_tile(w, 1, 0, 0,
                                                   blob1a, strlen(blob1a)));
    TEST_ASSERT_TRUE(arpt_archive_writer_add_tile(w, 1, 1, 0,
                                                   blob1b, strlen(blob1b)));
    TEST_ASSERT_TRUE(arpt_archive_writer_add_tile(w, 2, 2, 1,
                                                   blob2, strlen(blob2)));

    TEST_ASSERT_TRUE(arpt_archive_writer_finish(w));
    arpt_archive_writer_free(w);

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_EQUAL_UINT64(4, arpt_archive_reader_tile_count(r));

    /* Look up each tile */
    size_t size;
    const void *data;

    data = arpt_archive_reader_get_tile(r, 0, 0, 0, &size);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL_MEMORY(blob0, data, strlen(blob0));

    data = arpt_archive_reader_get_tile(r, 1, 0, 0, &size);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL_MEMORY(blob1a, data, strlen(blob1a));

    data = arpt_archive_reader_get_tile(r, 1, 1, 0, &size);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL_MEMORY(blob1b, data, strlen(blob1b));

    data = arpt_archive_reader_get_tile(r, 2, 2, 1, &size);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL_MEMORY(blob2, data, strlen(blob2));

    /* Non-existent tile */
    data = arpt_archive_reader_get_tile(r, 5, 10, 10, &size);
    TEST_ASSERT_NULL(data);
    TEST_ASSERT_EQUAL_size_t(0, size);

    arpt_archive_reader_close(r);
}

static void test_metadata(void) {
    arpt_archive_config cfg = test_config();
    arpt_archive_writer *w = arpt_archive_writer_create(&cfg);
    TEST_ASSERT_NOT_NULL(w);

    const char *meta = "{\"name\":\"test\"}";
    arpt_archive_writer_set_metadata(w, meta, strlen(meta));
    TEST_ASSERT_TRUE(arpt_archive_writer_add_tile(w, 0, 0, 0, "data", 4));
    TEST_ASSERT_TRUE(arpt_archive_writer_finish(w));
    arpt_archive_writer_free(w);

    /* Just verify the file can still be opened and read */
    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_EQUAL_UINT64(1, arpt_archive_reader_tile_count(r));
    arpt_archive_reader_close(r);
}

static void test_reader_nonexistent(void) {
    arpt_archive_reader *r = arpt_archive_reader_open("/nonexistent/file.arpa");
    TEST_ASSERT_NULL(r);
}

static void test_null_safety(void) {
    arpt_archive_writer_free(NULL);
    arpt_archive_reader_close(NULL);
    TEST_ASSERT_EQUAL_UINT64(0, arpt_archive_reader_tile_count(NULL));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_writer_create_free);
    RUN_TEST(test_empty_archive);
    RUN_TEST(test_single_tile);
    RUN_TEST(test_multiple_tiles);
    RUN_TEST(test_metadata);
    RUN_TEST(test_reader_nonexistent);
    RUN_TEST(test_null_safety);
    return UNITY_END();
}
