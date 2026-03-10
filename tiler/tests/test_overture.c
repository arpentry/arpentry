#include "unity.h"
#include "overture.h"
#include "wkb.h"
#include <carquet/carquet.h>
#include <stdio.h>
#include <string.h>

#define TEST_FILE "/tmp/arpt_test_overture.parquet"

void setUp(void) {}
void tearDown(void) {}

/* ── Null safety ────────────────────────────────────────────────────── */

static void test_overture_open_null(void) {
    TEST_ASSERT_NULL(arpt_overture_open(NULL));
}

static void test_overture_open_missing(void) {
    TEST_ASSERT_NULL(arpt_overture_open("/tmp/nonexistent_overture.parquet"));
}

static void test_overture_close_null(void) {
    arpt_overture_close(NULL);
}

static void test_overture_next_null(void) {
    arpt_overture_feature feat;
    TEST_ASSERT_FALSE(arpt_overture_next(NULL, &feat));
}

/* ── Helper: build WKB point ────────────────────────────────────────── */

static int32_t make_wkb_point(uint8_t *buf, double x, double y)
{
    buf[0] = 1;  /* LE */
    uint32_t type = 1;
    memcpy(buf + 1, &type, 4);
    memcpy(buf + 5, &x, 8);
    memcpy(buf + 13, &y, 8);
    return 21;
}

/* ── Helper: write synthetic GeoParquet ──────────────────────────────  */

static bool write_test_geoparquet(void) {
    carquet_error_t err = CARQUET_ERROR_INIT;

    /* Schema: geometry (BYTE_ARRAY), id (BYTE_ARRAY) */
    carquet_schema_t *schema = carquet_schema_create(&err);
    if (!schema) return false;

    carquet_schema_add_column(schema, "geometry",
        CARQUET_PHYSICAL_BYTE_ARRAY, NULL, CARQUET_REPETITION_REQUIRED, 0, 0);

    carquet_logical_type_t str_type = { .id = CARQUET_LOGICAL_STRING };
    carquet_schema_add_column(schema, "id",
        CARQUET_PHYSICAL_BYTE_ARRAY, &str_type, CARQUET_REPETITION_REQUIRED, 0, 0);

    carquet_writer_options_t opts;
    carquet_writer_options_init(&opts);

    carquet_writer_t *writer = carquet_writer_create(TEST_FILE, schema, &opts, &err);
    if (!writer) { carquet_schema_free(schema); return false; }

    /* Set GeoParquet metadata */
    carquet_writer_set_key_value(writer, "geo",
        "{\"primary_column\":\"geometry\","
        "\"columns\":{\"geometry\":{\"encoding\":\"WKB\"}}}");

    /* Write 3 rows */
    #define N_ROWS 3
    uint8_t wkb_bufs[N_ROWS][21];
    carquet_byte_array_t geom_vals[N_ROWS];
    carquet_byte_array_t id_vals[N_ROWS];
    char id_strs[N_ROWS][16];

    for (int i = 0; i < N_ROWS; i++) {
        int32_t len = make_wkb_point(wkb_bufs[i], (double)i * 10.0, (double)i * 20.0);
        geom_vals[i].data = wkb_bufs[i];
        geom_vals[i].length = len;

        snprintf(id_strs[i], sizeof(id_strs[i]), "feat_%d", i);
        id_vals[i].data = (uint8_t *)id_strs[i];
        id_vals[i].length = (int32_t)strlen(id_strs[i]);
    }

    carquet_writer_write_batch(writer, 0, geom_vals, N_ROWS, NULL, NULL);
    carquet_writer_write_batch(writer, 1, id_vals, N_ROWS, NULL, NULL);

    carquet_status_t st = carquet_writer_close(writer);
    carquet_schema_free(schema);
    return st == CARQUET_OK;
}

/* ── Roundtrip test ──────────────────────────────────────────────────  */

static void test_overture_roundtrip(void) {
    TEST_ASSERT_TRUE(write_test_geoparquet());

    arpt_overture *ov = arpt_overture_open(TEST_FILE);
    TEST_ASSERT_NOT_NULL(ov);

    int count = 0;
    arpt_overture_feature feat;
    while (arpt_overture_next(ov, &feat)) {
        TEST_ASSERT_EQUAL_UINT32(1, feat.geometry.type);  /* Point */
        TEST_ASSERT_EQUAL_UINT32(1, feat.geometry.n_coords);
        TEST_ASSERT_DOUBLE_WITHIN(0.01, (double)count * 10.0, feat.geometry.x[0]);
        TEST_ASSERT_DOUBLE_WITHIN(0.01, (double)count * 20.0, feat.geometry.y[0]);

        arpt_geom_free(&feat.geometry);
        count++;
    }
    TEST_ASSERT_EQUAL_INT(N_ROWS, count);

    arpt_overture_close(ov);
    remove(TEST_FILE);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_overture_open_null);
    RUN_TEST(test_overture_open_missing);
    RUN_TEST(test_overture_close_null);
    RUN_TEST(test_overture_next_null);
    RUN_TEST(test_overture_roundtrip);
    return UNITY_END();
}
