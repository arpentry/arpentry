#include "unity.h"
#include "archive.h"
#include "pipeline.h"

#include <stdio.h>
#include <string.h>

static const char *TEST_PATH = "/tmp/test_pipeline_output.arpa";

void setUp(void) {}
void tearDown(void) {
    remove(TEST_PATH);
}

static void test_pipeline_run_null(void) {
    TEST_ASSERT_FALSE(arpt_pipeline_run(NULL));
}

static void test_pipeline_run_no_output(void) {
    arpt_pipeline_config cfg = {0};
    cfg.synthetic = true;
    TEST_ASSERT_FALSE(arpt_pipeline_run(&cfg));
}

static void test_pipeline_synthetic_z0(void) {
    arpt_pipeline_config cfg = {
        .output = TEST_PATH,
        .tmp_dir = "/tmp",
        .mem_budget = 1024 * 1024,
        .bbox = {6.0, 46.0, 7.0, 47.0},
        .min_zoom = 0,
        .max_zoom = 0,
        .synthetic = true,
    };

    TEST_ASSERT_TRUE(arpt_pipeline_run(&cfg));

    /* Verify archive was created and has tiles */
    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_TRUE(arpt_archive_reader_tile_count(r) >= 1);
    arpt_archive_reader_close(r);
}

static void test_pipeline_synthetic_multi_zoom(void) {
    arpt_pipeline_config cfg = {
        .output = TEST_PATH,
        .tmp_dir = "/tmp",
        .mem_budget = 1024 * 1024,
        .bbox = {6.0, 46.0, 7.0, 47.0},
        .min_zoom = 0,
        .max_zoom = 3,
        .synthetic = true,
    };

    TEST_ASSERT_TRUE(arpt_pipeline_run(&cfg));

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    /* Multiple zoom levels should produce more tiles */
    uint64_t count = arpt_archive_reader_tile_count(r);
    TEST_ASSERT_TRUE(count > 1);
    arpt_archive_reader_close(r);
}

static void test_pipeline_synthetic_tile_readable(void) {
    arpt_pipeline_config cfg = {
        .output = TEST_PATH,
        .tmp_dir = "/tmp",
        .mem_budget = 1024 * 1024,
        .bbox = {6.0, 46.0, 7.0, 47.0},
        .min_zoom = 0,
        .max_zoom = 0,
        .synthetic = true,
    };

    TEST_ASSERT_TRUE(arpt_pipeline_run(&cfg));

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);

    /* z=0 tile (0,0) should exist and have non-zero size */
    size_t tile_size;
    const void *tile = arpt_archive_reader_get_tile(r, 0, 0, 0, &tile_size);
    TEST_ASSERT_NOT_NULL(tile);
    TEST_ASSERT_TRUE(tile_size > 0);

    arpt_archive_reader_close(r);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pipeline_run_null);
    RUN_TEST(test_pipeline_run_no_output);
    RUN_TEST(test_pipeline_synthetic_z0);
    RUN_TEST(test_pipeline_synthetic_multi_zoom);
    RUN_TEST(test_pipeline_synthetic_tile_readable);
    return UNITY_END();
}
