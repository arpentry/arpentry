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
    TEST_ASSERT_FALSE(arpt_pipeline_run(&cfg));
}

static void test_pipeline_empty_inputs_z0(void) {
    /* With no input files, the pipeline should still produce
       terrain-only tiles via empty tile fill. */
    arpt_pipeline_config cfg = {
        .output = TEST_PATH,
        .tmp_dir = "/tmp",
        .mem_budget = 1024 * 1024,
        .bbox = {6.0, 46.0, 7.0, 47.0},
        .min_zoom = 0,
        .max_zoom = 0,
        .inputs = NULL,
        .n_inputs = 0,
    };

    TEST_ASSERT_TRUE(arpt_pipeline_run(&cfg));

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_TRUE(arpt_archive_reader_tile_count(r) >= 1);
    arpt_archive_reader_close(r);
}

static void test_pipeline_empty_inputs_multi_zoom(void) {
    arpt_pipeline_config cfg = {
        .output = TEST_PATH,
        .tmp_dir = "/tmp",
        .mem_budget = 1024 * 1024,
        .bbox = {6.0, 46.0, 7.0, 47.0},
        .min_zoom = 0,
        .max_zoom = 3,
        .inputs = NULL,
        .n_inputs = 0,
    };

    TEST_ASSERT_TRUE(arpt_pipeline_run(&cfg));

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);
    uint64_t count = arpt_archive_reader_tile_count(r);
    TEST_ASSERT_TRUE(count > 1);
    arpt_archive_reader_close(r);
}

static void test_pipeline_empty_tile_readable(void) {
    arpt_pipeline_config cfg = {
        .output = TEST_PATH,
        .tmp_dir = "/tmp",
        .mem_budget = 1024 * 1024,
        .bbox = {6.0, 46.0, 7.0, 47.0},
        .min_zoom = 0,
        .max_zoom = 0,
        .inputs = NULL,
        .n_inputs = 0,
    };

    TEST_ASSERT_TRUE(arpt_pipeline_run(&cfg));

    arpt_archive_reader *r = arpt_archive_reader_open(TEST_PATH);
    TEST_ASSERT_NOT_NULL(r);

    /* z=0 tile (1,0) should exist (bbox 6-7 is in eastern hemisphere) */
    size_t tile_size;
    const void *tile = arpt_archive_reader_get_tile(r, 0, 1, 0, &tile_size);
    TEST_ASSERT_NOT_NULL(tile);
    TEST_ASSERT_TRUE(tile_size > 0);

    arpt_archive_reader_close(r);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pipeline_run_null);
    RUN_TEST(test_pipeline_run_no_output);
    RUN_TEST(test_pipeline_empty_inputs_z0);
    RUN_TEST(test_pipeline_empty_inputs_multi_zoom);
    RUN_TEST(test_pipeline_empty_tile_readable);
    return UNITY_END();
}
