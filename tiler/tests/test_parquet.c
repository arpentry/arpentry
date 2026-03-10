#include "unity.h"
#include "parquet.h"
#include <carquet/carquet.h>
#include <stdio.h>
#include <string.h>

#define TEST_FILE "/tmp/arpt_test_parquet.parquet"
#define NUM_ROWS 100

void setUp(void) {}
void tearDown(void) {}

/* ── Null safety tests ──────────────────────────────────────────────── */

static void test_parquet_open_null(void) {
    TEST_ASSERT_NULL(arpt_parquet_open(NULL));
}

static void test_parquet_open_missing_file(void) {
    TEST_ASSERT_NULL(arpt_parquet_open("/tmp/nonexistent_12345.parquet"));
}

static void test_parquet_close_null(void) {
    arpt_parquet_close(NULL);
}

static void test_parquet_num_rows_null(void) {
    TEST_ASSERT_EQUAL_INT64(0, arpt_parquet_num_rows(NULL));
}

static void test_parquet_num_columns_null(void) {
    TEST_ASSERT_EQUAL_INT32(0, arpt_parquet_num_columns(NULL));
}

static void test_parquet_find_column_null(void) {
    TEST_ASSERT_EQUAL_INT32(-1, arpt_parquet_find_column(NULL, "foo"));
}

static void test_parquet_cursor_null(void) {
    TEST_ASSERT_NULL(arpt_parquet_cursor_create(NULL, NULL, 0, 0));
}

static void test_parquet_cursor_next_null(void) {
    TEST_ASSERT_FALSE(arpt_parquet_cursor_next(NULL));
}

static void test_parquet_cursor_free_null(void) {
    arpt_parquet_cursor_free(NULL);
}

/* ── Helper: write a test parquet file using carquet directly ─────── */

static bool write_test_file(void) {
    carquet_error_t err = CARQUET_ERROR_INIT;

    carquet_schema_t *schema = carquet_schema_create(&err);
    if (!schema) return false;

    carquet_schema_add_column(schema, "id",
        CARQUET_PHYSICAL_INT32, NULL, CARQUET_REPETITION_REQUIRED, 0, 0);
    carquet_schema_add_column(schema, "value",
        CARQUET_PHYSICAL_DOUBLE, NULL, CARQUET_REPETITION_REQUIRED, 0, 0);

    carquet_logical_type_t string_type = { .id = CARQUET_LOGICAL_STRING };
    carquet_schema_add_column(schema, "name",
        CARQUET_PHYSICAL_BYTE_ARRAY, &string_type, CARQUET_REPETITION_REQUIRED, 0, 0);

    carquet_writer_options_t opts;
    carquet_writer_options_init(&opts);
    opts.compression = CARQUET_COMPRESSION_SNAPPY;

    carquet_writer_t *writer = carquet_writer_create(TEST_FILE, schema, &opts, &err);
    if (!writer) { carquet_schema_free(schema); return false; }

    int32_t ids[NUM_ROWS];
    double values[NUM_ROWS];
    carquet_byte_array_t names[NUM_ROWS];
    char name_buf[NUM_ROWS][16];

    for (int i = 0; i < NUM_ROWS; i++) {
        ids[i] = i;
        values[i] = (double)i * 1.5;
        snprintf(name_buf[i], sizeof(name_buf[i]), "row_%04d", i);
        names[i].data = (uint8_t *)name_buf[i];
        names[i].length = (int32_t)strlen(name_buf[i]);
    }

    carquet_writer_write_batch(writer, 0, ids, NUM_ROWS, NULL, NULL);
    carquet_writer_write_batch(writer, 1, values, NUM_ROWS, NULL, NULL);
    carquet_writer_write_batch(writer, 2, names, NUM_ROWS, NULL, NULL);

    carquet_status_t st = carquet_writer_close(writer);
    carquet_schema_free(schema);
    return st == CARQUET_OK;
}

/* ── Roundtrip tests ─────────────────────────────────────────────── */

static void test_parquet_roundtrip_metadata(void) {
    TEST_ASSERT_TRUE(write_test_file());

    arpt_parquet *pq = arpt_parquet_open(TEST_FILE);
    TEST_ASSERT_NOT_NULL(pq);

    TEST_ASSERT_EQUAL_INT64(NUM_ROWS, arpt_parquet_num_rows(pq));
    TEST_ASSERT_EQUAL_INT32(3, arpt_parquet_num_columns(pq));
    TEST_ASSERT_TRUE(arpt_parquet_num_row_groups(pq) >= 1);

    /* Column names */
    TEST_ASSERT_EQUAL_STRING("id", arpt_parquet_column_name(pq, 0));
    TEST_ASSERT_EQUAL_STRING("value", arpt_parquet_column_name(pq, 1));
    TEST_ASSERT_EQUAL_STRING("name", arpt_parquet_column_name(pq, 2));
    TEST_ASSERT_NULL(arpt_parquet_column_name(pq, 99));

    /* Column types */
    TEST_ASSERT_EQUAL(ARPT_PARQUET_INT32, arpt_parquet_column_type(pq, 0));
    TEST_ASSERT_EQUAL(ARPT_PARQUET_DOUBLE, arpt_parquet_column_type(pq, 1));
    TEST_ASSERT_EQUAL(ARPT_PARQUET_BYTES, arpt_parquet_column_type(pq, 2));

    /* Find column */
    TEST_ASSERT_EQUAL_INT32(0, arpt_parquet_find_column(pq, "id"));
    TEST_ASSERT_EQUAL_INT32(1, arpt_parquet_find_column(pq, "value"));
    TEST_ASSERT_EQUAL_INT32(2, arpt_parquet_find_column(pq, "name"));
    TEST_ASSERT_EQUAL_INT32(-1, arpt_parquet_find_column(pq, "missing"));

    arpt_parquet_close(pq);
    remove(TEST_FILE);
}

static void test_parquet_roundtrip_data(void) {
    TEST_ASSERT_TRUE(write_test_file());

    arpt_parquet *pq = arpt_parquet_open(TEST_FILE);
    TEST_ASSERT_NOT_NULL(pq);

    /* Read all columns */
    arpt_parquet_cursor *cur = arpt_parquet_cursor_create(pq, NULL, 0, 0);
    TEST_ASSERT_NOT_NULL(cur);

    int64_t total_rows = 0;
    while (arpt_parquet_cursor_next(cur)) {
        int64_t n = arpt_parquet_cursor_num_rows(cur);
        TEST_ASSERT_TRUE(n > 0);

        const int32_t *ids = arpt_parquet_cursor_data(cur, 0);
        const double *vals = arpt_parquet_cursor_data(cur, 1);
        TEST_ASSERT_NOT_NULL(ids);
        TEST_ASSERT_NOT_NULL(vals);

        /* Verify first row in this batch */
        int32_t expected_id = (int32_t)total_rows;
        TEST_ASSERT_EQUAL_INT32(expected_id, ids[0]);
        TEST_ASSERT_DOUBLE_WITHIN(0.001, (double)expected_id * 1.5, vals[0]);

        total_rows += n;
    }
    TEST_ASSERT_EQUAL_INT64(NUM_ROWS, total_rows);

    arpt_parquet_cursor_free(cur);
    arpt_parquet_close(pq);
    remove(TEST_FILE);
}

static void test_parquet_column_projection(void) {
    TEST_ASSERT_TRUE(write_test_file());

    arpt_parquet *pq = arpt_parquet_open(TEST_FILE);
    TEST_ASSERT_NOT_NULL(pq);

    /* Read only column 1 (value) */
    int32_t cols[] = {1};
    arpt_parquet_cursor *cur = arpt_parquet_cursor_create(pq, cols, 1, 0);
    TEST_ASSERT_NOT_NULL(cur);

    TEST_ASSERT_TRUE(arpt_parquet_cursor_next(cur));
    const double *vals = arpt_parquet_cursor_data(cur, 0);
    TEST_ASSERT_NOT_NULL(vals);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, vals[0]);

    arpt_parquet_cursor_free(cur);
    arpt_parquet_close(pq);
    remove(TEST_FILE);
}

/* ── KV metadata tests ───────────────────────────────────────────── */

static void test_parquet_key_value_metadata(void) {
    /* Write file with KV metadata */
    carquet_error_t err = CARQUET_ERROR_INIT;
    carquet_schema_t *schema = carquet_schema_create(&err);
    TEST_ASSERT_NOT_NULL(schema);

    carquet_schema_add_column(schema, "id",
        CARQUET_PHYSICAL_INT32, NULL, CARQUET_REPETITION_REQUIRED, 0, 0);

    carquet_writer_options_t opts;
    carquet_writer_options_init(&opts);

    carquet_writer_t *writer = carquet_writer_create(TEST_FILE, schema, &opts, &err);
    TEST_ASSERT_NOT_NULL(writer);

    carquet_writer_set_key_value(writer, "geo",
        "{\"primary_column\":\"geometry\"}");
    carquet_writer_set_key_value(writer, "version", "1.0");

    int32_t id = 1;
    carquet_writer_write_batch(writer, 0, &id, 1, NULL, NULL);
    carquet_writer_close(writer);
    carquet_schema_free(schema);

    /* Read back via tiler wrapper */
    arpt_parquet *pq = arpt_parquet_open(TEST_FILE);
    TEST_ASSERT_NOT_NULL(pq);

    TEST_ASSERT_EQUAL_INT32(2, arpt_parquet_num_key_values(pq));

    const char *geo = arpt_parquet_key_value(pq, "geo");
    TEST_ASSERT_NOT_NULL(geo);
    TEST_ASSERT_TRUE(strstr(geo, "geometry") != NULL);

    const char *ver = arpt_parquet_key_value(pq, "version");
    TEST_ASSERT_NOT_NULL(ver);
    TEST_ASSERT_EQUAL_STRING("1.0", ver);

    TEST_ASSERT_NULL(arpt_parquet_key_value(pq, "missing"));

    arpt_parquet_close(pq);
    remove(TEST_FILE);
}

static void test_parquet_num_key_values_null(void) {
    TEST_ASSERT_EQUAL_INT32(0, arpt_parquet_num_key_values(NULL));
}

static void test_parquet_find_column_path_null(void) {
    TEST_ASSERT_EQUAL_INT32(-1, arpt_parquet_find_column_path(NULL, "foo"));
}

static void test_parquet_find_column_path_flat(void) {
    TEST_ASSERT_TRUE(write_test_file());

    arpt_parquet *pq = arpt_parquet_open(TEST_FILE);
    TEST_ASSERT_NOT_NULL(pq);

    /* Flat column names should work as dot-paths */
    TEST_ASSERT_EQUAL_INT32(0, arpt_parquet_find_column_path(pq, "id"));
    TEST_ASSERT_EQUAL_INT32(1, arpt_parquet_find_column_path(pq, "value"));
    TEST_ASSERT_EQUAL_INT32(-1, arpt_parquet_find_column_path(pq, "missing"));

    arpt_parquet_close(pq);
    remove(TEST_FILE);
}

int main(void) {
    UNITY_BEGIN();
    /* Null safety */
    RUN_TEST(test_parquet_open_null);
    RUN_TEST(test_parquet_open_missing_file);
    RUN_TEST(test_parquet_close_null);
    RUN_TEST(test_parquet_num_rows_null);
    RUN_TEST(test_parquet_num_columns_null);
    RUN_TEST(test_parquet_find_column_null);
    RUN_TEST(test_parquet_cursor_null);
    RUN_TEST(test_parquet_cursor_next_null);
    RUN_TEST(test_parquet_cursor_free_null);
    RUN_TEST(test_parquet_num_key_values_null);
    RUN_TEST(test_parquet_find_column_path_null);
    /* Roundtrip */
    RUN_TEST(test_parquet_roundtrip_metadata);
    RUN_TEST(test_parquet_roundtrip_data);
    RUN_TEST(test_parquet_column_projection);
    /* KV metadata */
    RUN_TEST(test_parquet_key_value_metadata);
    RUN_TEST(test_parquet_find_column_path_flat);
    return UNITY_END();
}
