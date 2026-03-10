#include "unity.h"
#include "geoparquet.h"
#include <string.h>
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* ── Null safety ────────────────────────────────────────────────────── */

static void test_geoparquet_parse_null(void) {
    arpt_geoparquet_meta meta;
    TEST_ASSERT_FALSE(arpt_geoparquet_parse(NULL, &meta));
    TEST_ASSERT_FALSE(arpt_geoparquet_parse("{}", NULL));
}

/* ── Minimal valid JSON ─────────────────────────────────────────────── */

static void test_geoparquet_parse_minimal(void) {
    const char *json =
        "{\"primary_column\":\"geometry\","
        "\"columns\":{\"geometry\":{\"encoding\":\"WKB\"}}}";

    arpt_geoparquet_meta meta;
    TEST_ASSERT_TRUE(arpt_geoparquet_parse(json, &meta));
    TEST_ASSERT_EQUAL_STRING("geometry", meta.primary_column);
    TEST_ASSERT_EQUAL_STRING("WKB", meta.encoding);
    TEST_ASSERT_FALSE(meta.has_bbox);
}

/* ── Full metadata with bbox ────────────────────────────────────────── */

static void test_geoparquet_parse_with_bbox(void) {
    const char *json =
        "{\"primary_column\":\"geom\","
        "\"columns\":{\"geom\":{"
        "\"encoding\":\"WKB\","
        "\"bbox\":[-180.0,-90.0,180.0,90.0]}}}";

    arpt_geoparquet_meta meta;
    TEST_ASSERT_TRUE(arpt_geoparquet_parse(json, &meta));
    TEST_ASSERT_EQUAL_STRING("geom", meta.primary_column);
    TEST_ASSERT_EQUAL_STRING("WKB", meta.encoding);
    TEST_ASSERT_TRUE(meta.has_bbox);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -180.0, meta.bbox[0]);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -90.0, meta.bbox[1]);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 180.0, meta.bbox[2]);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 90.0, meta.bbox[3]);
}

/* ── Defaults when fields are missing ───────────────────────────────── */

static void test_geoparquet_parse_defaults(void) {
    const char *json = "{\"columns\":{\"geometry\":{}}}";

    arpt_geoparquet_meta meta;
    TEST_ASSERT_TRUE(arpt_geoparquet_parse(json, &meta));
    TEST_ASSERT_EQUAL_STRING("geometry", meta.primary_column);
    TEST_ASSERT_EQUAL_STRING("WKB", meta.encoding);
    TEST_ASSERT_FALSE(meta.has_bbox);
}

/* ── Empty JSON object ──────────────────────────────────────────────── */

static void test_geoparquet_parse_empty(void) {
    arpt_geoparquet_meta meta;
    TEST_ASSERT_TRUE(arpt_geoparquet_parse("{}", &meta));
    TEST_ASSERT_EQUAL_STRING("geometry", meta.primary_column);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_geoparquet_parse_null);
    RUN_TEST(test_geoparquet_parse_minimal);
    RUN_TEST(test_geoparquet_parse_with_bbox);
    RUN_TEST(test_geoparquet_parse_defaults);
    RUN_TEST(test_geoparquet_parse_empty);
    return UNITY_END();
}
