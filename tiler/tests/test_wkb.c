#include "unity.h"
#include "wkb.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* ── Helpers: build WKB byte arrays ─────────────────────────────────── */

static size_t put_byte(uint8_t *buf, size_t pos, uint8_t val)
{
    buf[pos] = val;
    return pos + 1;
}

static size_t put_u32_le(uint8_t *buf, size_t pos, uint32_t val)
{
    buf[pos + 0] = (uint8_t)(val & 0xFF);
    buf[pos + 1] = (uint8_t)((val >> 8) & 0xFF);
    buf[pos + 2] = (uint8_t)((val >> 16) & 0xFF);
    buf[pos + 3] = (uint8_t)((val >> 24) & 0xFF);
    return pos + 4;
}

static size_t put_u32_be(uint8_t *buf, size_t pos, uint32_t val)
{
    buf[pos + 0] = (uint8_t)((val >> 24) & 0xFF);
    buf[pos + 1] = (uint8_t)((val >> 16) & 0xFF);
    buf[pos + 2] = (uint8_t)((val >> 8) & 0xFF);
    buf[pos + 3] = (uint8_t)(val & 0xFF);
    return pos + 4;
}

static size_t put_f64_le(uint8_t *buf, size_t pos, double val)
{
    memcpy(buf + pos, &val, 8);
    return pos + 8;
}

static size_t put_f64_be(uint8_t *buf, size_t pos, double val)
{
    uint8_t tmp[8];
    memcpy(tmp, &val, 8);
    for (int i = 0; i < 8; i++) buf[pos + i] = tmp[7 - i];
    return pos + 8;
}

/* ── Null safety ────────────────────────────────────────────────────── */

static void test_wkb_parse_null(void) {
    arpt_geom g = {0};
    TEST_ASSERT_FALSE(arpt_wkb_parse(NULL, 0, &g));
}

static void test_wkb_parse_too_short(void) {
    uint8_t data[3] = {1, 1, 0};
    arpt_geom g = {0};
    TEST_ASSERT_FALSE(arpt_wkb_parse(data, 3, &g));
}

/* ── Point (2D, little-endian) ──────────────────────────────────────── */

static void test_wkb_point_2d_le(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 1);           /* LE */
    p = put_u32_le(buf, p, 1);         /* Point */
    p = put_f64_le(buf, p, 1.5);       /* x */
    p = put_f64_le(buf, p, 2.5);       /* y */

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(1, g.type);
    TEST_ASSERT_EQUAL_UINT32(1, g.n_coords);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.5, g.x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 2.5, g.y[0]);
    TEST_ASSERT_NULL(g.z);
    arpt_geom_free(&g);
}

/* ── Point (2D, big-endian) ─────────────────────────────────────────── */

static void test_wkb_point_2d_be(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 0);           /* BE */
    p = put_u32_be(buf, p, 1);         /* Point */
    p = put_f64_be(buf, p, 3.0);       /* x */
    p = put_f64_be(buf, p, 4.0);       /* y */

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(1, g.type);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 3.0, g.x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 4.0, g.y[0]);
    TEST_ASSERT_NULL(g.z);
    arpt_geom_free(&g);
}

/* ── Point (3D ISO, little-endian) ──────────────────────────────────── */

static void test_wkb_point_3d_iso(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 1);           /* LE */
    p = put_u32_le(buf, p, 1001);      /* Point Z (ISO) */
    p = put_f64_le(buf, p, 1.0);
    p = put_f64_le(buf, p, 2.0);
    p = put_f64_le(buf, p, 3.0);       /* z */

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(1, g.type);
    TEST_ASSERT_NOT_NULL(g.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 3.0, g.z[0]);
    arpt_geom_free(&g);
}

/* ── Point (3D OGC, little-endian) ──────────────────────────────────── */

static void test_wkb_point_3d_ogc(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 1);               /* LE */
    p = put_u32_le(buf, p, 0x80000001u);   /* Point Z (OGC) */
    p = put_f64_le(buf, p, 10.0);
    p = put_f64_le(buf, p, 20.0);
    p = put_f64_le(buf, p, 30.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(1, g.type);
    TEST_ASSERT_NOT_NULL(g.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 30.0, g.z[0]);
    arpt_geom_free(&g);
}

/* ── LineString (2D) ────────────────────────────────────────────────── */

static void test_wkb_linestring_2d(void) {
    uint8_t buf[128];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 2);         /* LineString */
    p = put_u32_le(buf, p, 3);         /* 3 points */
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 1.0);
    p = put_f64_le(buf, p, 2.0); p = put_f64_le(buf, p, 2.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(2, g.type);
    TEST_ASSERT_EQUAL_UINT32(3, g.n_coords);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 2.0, g.x[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 2.0, g.y[2]);
    TEST_ASSERT_NULL(g.z);
    arpt_geom_free(&g);
}

/* ── LineString (3D ISO) ────────────────────────────────────────────── */

static void test_wkb_linestring_3d(void) {
    uint8_t buf[256];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 1002);      /* LineString Z (ISO) */
    p = put_u32_le(buf, p, 2);         /* 2 points */
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 10.0);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 20.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(2, g.type);
    TEST_ASSERT_NOT_NULL(g.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 20.0, g.z[1]);
    arpt_geom_free(&g);
}

/* ── Polygon (2D) — triangle ────────────────────────────────────────── */

static void test_wkb_polygon_2d(void) {
    uint8_t buf[256];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 3);         /* Polygon */
    p = put_u32_le(buf, p, 1);         /* 1 ring */
    p = put_u32_le(buf, p, 4);         /* 4 points (closed) */
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 1.0);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(3, g.type);
    TEST_ASSERT_EQUAL_UINT32(4, g.n_coords);
    TEST_ASSERT_EQUAL_UINT32(2, g.n_offsets); /* N+1 sentinel style */
    TEST_ASSERT_EQUAL_UINT32(0, g.offsets[0]);
    TEST_ASSERT_EQUAL_UINT32(4, g.offsets[1]); /* sentinel */
    arpt_geom_free(&g);
}

/* ── Polygon with hole ──────────────────────────────────────────────── */

static void test_wkb_polygon_with_hole(void) {
    uint8_t buf[512];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 3);         /* Polygon */
    p = put_u32_le(buf, p, 2);         /* 2 rings */

    /* Outer ring: 4 points */
    p = put_u32_le(buf, p, 4);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 10.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 10.0); p = put_f64_le(buf, p, 10.0);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);

    /* Inner ring (hole): 4 points */
    p = put_u32_le(buf, p, 4);
    p = put_f64_le(buf, p, 2.0); p = put_f64_le(buf, p, 2.0);
    p = put_f64_le(buf, p, 8.0); p = put_f64_le(buf, p, 2.0);
    p = put_f64_le(buf, p, 8.0); p = put_f64_le(buf, p, 8.0);
    p = put_f64_le(buf, p, 2.0); p = put_f64_le(buf, p, 2.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(3, g.type);
    TEST_ASSERT_EQUAL_UINT32(8, g.n_coords);
    TEST_ASSERT_EQUAL_UINT32(3, g.n_offsets); /* N+1 sentinel style */
    TEST_ASSERT_EQUAL_UINT32(0, g.offsets[0]);
    TEST_ASSERT_EQUAL_UINT32(4, g.offsets[1]);
    TEST_ASSERT_EQUAL_UINT32(8, g.offsets[2]); /* sentinel */
    arpt_geom_free(&g);
}

/* ── MultiPoint ─────────────────────────────────────────────────────── */

static void test_wkb_multipoint(void) {
    uint8_t buf[256];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 4);         /* MultiPoint */
    p = put_u32_le(buf, p, 2);         /* 2 points */

    /* Point 1 */
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 1);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 2.0);

    /* Point 2 */
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 1);
    p = put_f64_le(buf, p, 3.0); p = put_f64_le(buf, p, 4.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(4, g.type);
    TEST_ASSERT_EQUAL_UINT32(2, g.n_coords);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 3.0, g.x[1]);
    arpt_geom_free(&g);
}

/* ── MultiLineString ────────────────────────────────────────────────── */

static void test_wkb_multilinestring(void) {
    uint8_t buf[512];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 5);         /* MultiLineString */
    p = put_u32_le(buf, p, 2);         /* 2 linestrings */

    /* LineString 1: 2 points */
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 2);
    p = put_u32_le(buf, p, 2);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 1.0);

    /* LineString 2: 2 points */
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 2);
    p = put_u32_le(buf, p, 2);
    p = put_f64_le(buf, p, 2.0); p = put_f64_le(buf, p, 2.0);
    p = put_f64_le(buf, p, 3.0); p = put_f64_le(buf, p, 3.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(5, g.type);
    TEST_ASSERT_EQUAL_UINT32(4, g.n_coords);
    TEST_ASSERT_EQUAL_UINT32(3, g.n_offsets); /* N+1 sentinel style */
    TEST_ASSERT_EQUAL_UINT32(0, g.offsets[0]);
    TEST_ASSERT_EQUAL_UINT32(2, g.offsets[1]);
    TEST_ASSERT_EQUAL_UINT32(4, g.offsets[2]); /* sentinel */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 3.0, g.x[3]);
    arpt_geom_free(&g);
}

/* ── MultiPolygon ───────────────────────────────────────────────────── */

static void test_wkb_multipolygon(void) {
    uint8_t buf[512];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 6);         /* MultiPolygon */
    p = put_u32_le(buf, p, 2);         /* 2 polygons */

    /* Polygon 1: 1 ring, 4 points */
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 3);         /* Polygon */
    p = put_u32_le(buf, p, 1);
    p = put_u32_le(buf, p, 4);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 0.0);
    p = put_f64_le(buf, p, 1.0); p = put_f64_le(buf, p, 1.0);
    p = put_f64_le(buf, p, 0.0); p = put_f64_le(buf, p, 0.0);

    /* Polygon 2: 1 ring, 4 points */
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 3);         /* Polygon */
    p = put_u32_le(buf, p, 1);
    p = put_u32_le(buf, p, 4);
    p = put_f64_le(buf, p, 5.0); p = put_f64_le(buf, p, 5.0);
    p = put_f64_le(buf, p, 6.0); p = put_f64_le(buf, p, 5.0);
    p = put_f64_le(buf, p, 6.0); p = put_f64_le(buf, p, 6.0);
    p = put_f64_le(buf, p, 5.0); p = put_f64_le(buf, p, 5.0);

    arpt_geom g = {0};
    TEST_ASSERT_TRUE(arpt_wkb_parse(buf, p, &g));
    TEST_ASSERT_EQUAL_UINT32(6, g.type);
    TEST_ASSERT_EQUAL_UINT32(8, g.n_coords);
    TEST_ASSERT_EQUAL_UINT32(3, g.n_offsets);  /* N+1 sentinel style: 2 rings + sentinel */
    TEST_ASSERT_EQUAL_UINT32(2, g.n_parts);    /* 2 polygons */
    TEST_ASSERT_EQUAL_UINT32(0, g.parts[0]);
    TEST_ASSERT_EQUAL_UINT32(1, g.parts[1]);
    TEST_ASSERT_EQUAL_UINT32(0, g.offsets[0]);
    TEST_ASSERT_EQUAL_UINT32(4, g.offsets[1]);
    TEST_ASSERT_EQUAL_UINT32(8, g.offsets[2]); /* sentinel */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 5.0, g.x[4]);
    arpt_geom_free(&g);
}

/* ── Truncated input ────────────────────────────────────────────────── */

static void test_wkb_truncated_point(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 1);
    p = put_f64_le(buf, p, 1.0);
    /* Missing y coordinate */

    arpt_geom g = {0};
    TEST_ASSERT_FALSE(arpt_wkb_parse(buf, p, &g));
}

static void test_wkb_truncated_linestring(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 2);
    p = put_u32_le(buf, p, 10);        /* Claims 10 points */
    p = put_f64_le(buf, p, 1.0);       /* Only partial data */

    arpt_geom g = {0};
    TEST_ASSERT_FALSE(arpt_wkb_parse(buf, p, &g));
}

/* ── Invalid type ───────────────────────────────────────────────────── */

static void test_wkb_invalid_type(void) {
    uint8_t buf[64];
    size_t p = 0;
    p = put_byte(buf, p, 1);
    p = put_u32_le(buf, p, 99);        /* Unknown type */

    arpt_geom g = {0};
    TEST_ASSERT_FALSE(arpt_wkb_parse(buf, p, &g));
}

int main(void) {
    UNITY_BEGIN();

    /* Null safety */
    RUN_TEST(test_wkb_parse_null);
    RUN_TEST(test_wkb_parse_too_short);

    /* Point */
    RUN_TEST(test_wkb_point_2d_le);
    RUN_TEST(test_wkb_point_2d_be);
    RUN_TEST(test_wkb_point_3d_iso);
    RUN_TEST(test_wkb_point_3d_ogc);

    /* LineString */
    RUN_TEST(test_wkb_linestring_2d);
    RUN_TEST(test_wkb_linestring_3d);

    /* Polygon */
    RUN_TEST(test_wkb_polygon_2d);
    RUN_TEST(test_wkb_polygon_with_hole);

    /* Multi */
    RUN_TEST(test_wkb_multipoint);
    RUN_TEST(test_wkb_multilinestring);
    RUN_TEST(test_wkb_multipolygon);

    /* Error cases */
    RUN_TEST(test_wkb_truncated_point);
    RUN_TEST(test_wkb_truncated_linestring);
    RUN_TEST(test_wkb_invalid_type);

    return UNITY_END();
}
