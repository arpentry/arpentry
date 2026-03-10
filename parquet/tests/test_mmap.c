/**
 * @file test_mmap.c
 * @brief Tests for memory-mapped I/O and zero-copy reading (from carquet upstream)
 */

#include <carquet/carquet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_FAIL(name, msg) do { printf("[FAIL] %s: %s\n", name, msg); return 1; } while(0)

#define TEST_FILE "/tmp/carquet_test_mmap.parquet"

/* ============================================================================
 * Helper: Create a test file with uncompressed data
 * ============================================================================
 */

static int create_test_file(int64_t num_rows) {
    carquet_error_t error = CARQUET_ERROR_INIT;

    carquet_schema_t* schema = carquet_schema_create(&error);
    if (!schema) return 1;

    carquet_schema_add_column(schema, "id", CARQUET_PHYSICAL_INT64,
                               NULL, CARQUET_REPETITION_REQUIRED, 0, 0);
    carquet_schema_add_column(schema, "value", CARQUET_PHYSICAL_DOUBLE,
                               NULL, CARQUET_REPETITION_REQUIRED, 0, 0);

    carquet_writer_options_t opts;
    carquet_writer_options_init(&opts);
    opts.compression = CARQUET_COMPRESSION_UNCOMPRESSED;
    opts.row_group_size = num_rows;

    carquet_writer_t* writer = carquet_writer_create(TEST_FILE, schema, &opts, &error);
    if (!writer) {
        carquet_schema_free(schema);
        return 1;
    }

    int64_t* ids = malloc(sizeof(int64_t) * (size_t)num_rows);
    double* values = malloc(sizeof(double) * (size_t)num_rows);

    for (int64_t i = 0; i < num_rows; i++) {
        ids[i] = i * 100;
        values[i] = (double)i * 3.14159;
    }

    carquet_writer_write_batch(writer, 0, ids, num_rows, NULL, NULL);
    carquet_writer_write_batch(writer, 1, values, num_rows, NULL, NULL);

    carquet_writer_close(writer);
    carquet_schema_free(schema);
    free(ids);
    free(values);

    return 0;
}

/* ============================================================================
 * Tests
 * ============================================================================
 */

static int test_mmap_open(void) {
    carquet_error_t error = CARQUET_ERROR_INIT;

    if (create_test_file(1000) != 0) {
        TEST_FAIL("mmap_open", "Failed to create test file");
    }

    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);
    opts.use_mmap = true;

    carquet_reader_t* reader = carquet_reader_open(TEST_FILE, &opts, &error);
    if (!reader) {
        TEST_FAIL("mmap_open", "Failed to open reader with mmap");
    }

    if (!carquet_reader_is_mmap(reader)) {
        carquet_reader_close(reader);
        TEST_FAIL("mmap_open", "mmap should be active");
    }

    if (carquet_reader_num_rows(reader) != 1000) {
        carquet_reader_close(reader);
        TEST_FAIL("mmap_open", "Wrong row count");
    }

    if (carquet_reader_num_columns(reader) != 2) {
        carquet_reader_close(reader);
        TEST_FAIL("mmap_open", "Wrong column count");
    }

    carquet_reader_close(reader);
    TEST_PASS("mmap_open");
    return 0;
}

static int test_zero_copy_eligibility(void) {
    carquet_error_t error = CARQUET_ERROR_INIT;

    if (create_test_file(1000) != 0) {
        TEST_FAIL("zero_copy_eligibility", "Failed to create test file");
    }

    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);
    opts.use_mmap = true;

    carquet_reader_t* reader = carquet_reader_open(TEST_FILE, &opts, &error);
    if (!reader) {
        TEST_FAIL("zero_copy_eligibility", "Failed to open reader");
    }

    if (!carquet_reader_can_zero_copy(reader, 0, 0)) {
        carquet_reader_close(reader);
        TEST_FAIL("zero_copy_eligibility", "INT64 column should be zero-copy eligible");
    }

    if (!carquet_reader_can_zero_copy(reader, 0, 1)) {
        carquet_reader_close(reader);
        TEST_FAIL("zero_copy_eligibility", "DOUBLE column should be zero-copy eligible");
    }

    carquet_reader_close(reader);
    TEST_PASS("zero_copy_eligibility");
    return 0;
}

static int test_mmap_read_data(void) {
    carquet_error_t error = CARQUET_ERROR_INIT;
    int64_t num_rows = 1000;

    if (create_test_file(num_rows) != 0) {
        TEST_FAIL("mmap_read_data", "Failed to create test file");
    }

    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);
    opts.use_mmap = true;

    carquet_reader_t* reader = carquet_reader_open(TEST_FILE, &opts, &error);
    if (!reader) {
        TEST_FAIL("mmap_read_data", "Failed to open reader");
    }

    carquet_column_reader_t* col_reader = carquet_reader_get_column(reader, 0, 0, &error);
    if (!col_reader) {
        carquet_reader_close(reader);
        TEST_FAIL("mmap_read_data", "Failed to get column reader");
    }

    int64_t* data = malloc(sizeof(int64_t) * (size_t)num_rows);
    int64_t values_read = carquet_column_read_batch(col_reader, data, num_rows, NULL, NULL);

    if (values_read != num_rows) {
        free(data);
        carquet_column_reader_free(col_reader);
        carquet_reader_close(reader);
        TEST_FAIL("mmap_read_data", "Wrong number of values read");
    }

    for (int64_t i = 0; i < num_rows; i++) {
        if (data[i] != i * 100) {
            free(data);
            carquet_column_reader_free(col_reader);
            carquet_reader_close(reader);
            TEST_FAIL("mmap_read_data", "Data mismatch");
        }
    }

    free(data);
    carquet_column_reader_free(col_reader);
    carquet_reader_close(reader);
    TEST_PASS("mmap_read_data");
    return 0;
}

static int test_mmap_batch_reader(void) {
    carquet_error_t error = CARQUET_ERROR_INIT;
    int64_t num_rows = 1000;

    if (create_test_file(num_rows) != 0) {
        TEST_FAIL("mmap_batch_reader", "Failed to create test file");
    }

    carquet_reader_options_t reader_opts;
    carquet_reader_options_init(&reader_opts);
    reader_opts.use_mmap = true;

    carquet_reader_t* reader = carquet_reader_open(TEST_FILE, &reader_opts, &error);
    if (!reader) {
        TEST_FAIL("mmap_batch_reader", "Failed to open reader");
    }

    carquet_batch_reader_config_t config;
    carquet_batch_reader_config_init(&config);
    config.batch_size = num_rows;

    carquet_batch_reader_t* batch_reader = carquet_batch_reader_create(reader, &config, &error);
    if (!batch_reader) {
        carquet_reader_close(reader);
        TEST_FAIL("mmap_batch_reader", "Failed to create batch reader");
    }

    carquet_row_batch_t* batch = NULL;
    carquet_status_t status = carquet_batch_reader_next(batch_reader, &batch);
    if (status != CARQUET_OK || !batch) {
        carquet_batch_reader_free(batch_reader);
        carquet_reader_close(reader);
        TEST_FAIL("mmap_batch_reader", "Failed to read batch");
    }

    if (carquet_row_batch_num_rows(batch) != num_rows) {
        carquet_row_batch_free(batch);
        carquet_batch_reader_free(batch_reader);
        carquet_reader_close(reader);
        TEST_FAIL("mmap_batch_reader", "Wrong batch row count");
    }

    const void* data;
    const uint8_t* null_bitmap;
    int64_t col_num_values;

    status = carquet_row_batch_column(batch, 0, &data, &null_bitmap, &col_num_values);
    if (status != CARQUET_OK) {
        carquet_row_batch_free(batch);
        carquet_batch_reader_free(batch_reader);
        carquet_reader_close(reader);
        TEST_FAIL("mmap_batch_reader", "Failed to get column data");
    }

    const int64_t* int_data = (const int64_t*)data;
    for (int64_t i = 0; i < num_rows; i++) {
        if (int_data[i] != i * 100) {
            carquet_row_batch_free(batch);
            carquet_batch_reader_free(batch_reader);
            carquet_reader_close(reader);
            TEST_FAIL("mmap_batch_reader", "Data mismatch in batch");
        }
    }

    carquet_row_batch_free(batch);
    carquet_batch_reader_free(batch_reader);
    carquet_reader_close(reader);
    TEST_PASS("mmap_batch_reader");
    return 0;
}

static int test_mmap_vs_fread(void) {
    carquet_error_t error = CARQUET_ERROR_INIT;
    int64_t num_rows = 5000;

    if (create_test_file(num_rows) != 0) {
        TEST_FAIL("mmap_vs_fread", "Failed to create test file");
    }

    /* Read with mmap */
    carquet_reader_options_t mmap_opts;
    carquet_reader_options_init(&mmap_opts);
    mmap_opts.use_mmap = true;

    carquet_reader_t* mmap_reader = carquet_reader_open(TEST_FILE, &mmap_opts, &error);
    if (!mmap_reader) {
        TEST_FAIL("mmap_vs_fread", "Failed to open mmap reader");
    }

    carquet_column_reader_t* mmap_col = carquet_reader_get_column(mmap_reader, 0, 0, &error);
    int64_t* mmap_data = malloc(sizeof(int64_t) * (size_t)num_rows);
    carquet_column_read_batch(mmap_col, mmap_data, num_rows, NULL, NULL);

    carquet_column_reader_free(mmap_col);
    carquet_reader_close(mmap_reader);

    /* Read with fread */
    carquet_reader_options_t fread_opts;
    carquet_reader_options_init(&fread_opts);
    fread_opts.use_mmap = false;

    carquet_reader_t* fread_reader = carquet_reader_open(TEST_FILE, &fread_opts, &error);
    if (!fread_reader) {
        free(mmap_data);
        TEST_FAIL("mmap_vs_fread", "Failed to open fread reader");
    }

    carquet_column_reader_t* fread_col = carquet_reader_get_column(fread_reader, 0, 0, &error);
    int64_t* fread_data = malloc(sizeof(int64_t) * (size_t)num_rows);
    carquet_column_read_batch(fread_col, fread_data, num_rows, NULL, NULL);

    carquet_column_reader_free(fread_col);
    carquet_reader_close(fread_reader);

    /* Compare results */
    int mismatch = 0;
    for (int64_t i = 0; i < num_rows; i++) {
        if (mmap_data[i] != fread_data[i]) {
            mismatch = 1;
            break;
        }
    }

    free(mmap_data);
    free(fread_data);

    if (mismatch) {
        TEST_FAIL("mmap_vs_fread", "mmap and fread results differ");
    }

    TEST_PASS("mmap_vs_fread");
    return 0;
}

static int test_fread_fallback(void) {
    carquet_error_t error = CARQUET_ERROR_INIT;

    if (create_test_file(100) != 0) {
        TEST_FAIL("fread_fallback", "Failed to create test file");
    }

    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);
    opts.use_mmap = false;

    carquet_reader_t* reader = carquet_reader_open(TEST_FILE, &opts, &error);
    if (!reader) {
        TEST_FAIL("fread_fallback", "Failed to open reader");
    }

    if (carquet_reader_is_mmap(reader)) {
        carquet_reader_close(reader);
        TEST_FAIL("fread_fallback", "mmap should NOT be active");
    }

    if (carquet_reader_can_zero_copy(reader, 0, 0)) {
        carquet_reader_close(reader);
        TEST_FAIL("fread_fallback", "zero-copy should not be possible without mmap");
    }

    carquet_reader_close(reader);
    TEST_PASS("fread_fallback");
    return 0;
}

int main(void) {
    printf("=== Memory-Mapped I/O Tests ===\n\n");

    int failures = 0;

    failures += test_mmap_open();
    failures += test_zero_copy_eligibility();
    failures += test_mmap_read_data();
    failures += test_mmap_batch_reader();
    failures += test_mmap_vs_fread();
    failures += test_fread_fallback();

    /* Cleanup */
    remove(TEST_FILE);

    printf("\n=== Results: %d failures ===\n", failures);
    return failures > 0 ? 1 : 0;
}
