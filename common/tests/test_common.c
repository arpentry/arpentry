#include "unity.h"
#include "arpentry/common.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* ── Existing low-level quantization tests ──────────────────────────────── */

void test_dequantize_tile_origin(void) {
    /* q = 16384 → origin (0.0) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, arpt_dequantize(16384));
}

void test_dequantize_tile_end(void) {
    /* q = 49151 → just under 1.0 */
    double v = arpt_dequantize(49151);
    TEST_ASSERT_TRUE(v > 0.99 && v < 1.0);
}

void test_quantize_roundtrip(void) {
    /* Quantize then dequantize should round-trip within 1 quantum */
    double original = 0.42;
    uint16_t q = arpt_quantize(original);
    double recovered = arpt_dequantize(q);
    double quantum = 1.0 / 32768.0;
    TEST_ASSERT_DOUBLE_WITHIN(quantum, original, recovered);
}

void test_quantize_clamps_low(void) {
    TEST_ASSERT_EQUAL_UINT16(0, arpt_quantize(-1.0));
}

void test_quantize_clamps_high(void) {
    TEST_ASSERT_EQUAL_UINT16(65535, arpt_quantize(2.0));
}

/* ── Tile bounds ────────────────────────────────────────────────────────── */

void test_tile_bounds_root_west(void) {
    /* Level 0, tile (0,0): western hemisphere */
    arpt_bounds_t b = arpt_tile_bounds(0, 0, 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -180.0, b.west);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9,    0.0, b.east);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9,  -90.0, b.south);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9,   90.0, b.north);
}

void test_tile_bounds_root_east(void) {
    /* Level 0, tile (1,0): eastern hemisphere */
    arpt_bounds_t b = arpt_tile_bounds(0, 1, 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9,   0.0, b.west);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 180.0, b.east);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -90.0, b.south);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9,  90.0, b.north);
}

void test_tile_bounds_level5(void) {
    /* Level 5, tile (32, 16): should be in a known range */
    arpt_bounds_t b = arpt_tile_bounds(5, 32, 16);
    double lon_span = 360.0 / 64.0;  /* 2^(5+1) = 64 */
    double lat_span = 180.0 / 32.0;  /* 2^5 = 32 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -180.0 + 32 * lon_span, b.west);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -180.0 + 33 * lon_span, b.east);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -90.0 + 16 * lat_span, b.south);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -90.0 + 17 * lat_span, b.north);
}

/* ── Geodetic quantization ──────────────────────────────────────────────── */

void test_geodetic_quantize_lon_roundtrip(void) {
    arpt_bounds_t b = arpt_tile_bounds(10, 500, 300);
    double lon = (b.west + b.east) / 2.0; /* tile center */
    uint16_t q = arpt_quantize_lon(lon, b);
    double recovered = arpt_dequantize_lon(q, b);
    /* Precision: tile width / 32768 */
    double precision = (b.east - b.west) / 32768.0;
    TEST_ASSERT_DOUBLE_WITHIN(precision, lon, recovered);
}

void test_geodetic_quantize_lat_roundtrip(void) {
    arpt_bounds_t b = arpt_tile_bounds(10, 500, 300);
    double lat = (b.south + b.north) / 2.0;
    uint16_t q = arpt_quantize_lat(lat, b);
    double recovered = arpt_dequantize_lat(q, b);
    double precision = (b.north - b.south) / 32768.0;
    TEST_ASSERT_DOUBLE_WITHIN(precision, lat, recovered);
}

void test_geodetic_quantize_tile_origin(void) {
    /* Tile origin should quantize to ARPT_BUFFER (16384) */
    arpt_bounds_t b = arpt_tile_bounds(5, 10, 8);
    uint16_t qx = arpt_quantize_lon(b.west, b);
    uint16_t qy = arpt_quantize_lat(b.south, b);
    TEST_ASSERT_EQUAL_UINT16(ARPT_BUFFER, qx);
    TEST_ASSERT_EQUAL_UINT16(ARPT_BUFFER, qy);
}

/* ── Elevation ──────────────────────────────────────────────────────────── */

void test_elevation_roundtrip(void) {
    double meters = 4478.123;  /* Matterhorn-ish */
    int32_t mm = arpt_meters_to_mm(meters);
    double recovered = arpt_mm_to_meters(mm);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, meters, recovered);
}

void test_elevation_negative(void) {
    double meters = -422.0;  /* Dead Sea-ish */
    int32_t mm = arpt_meters_to_mm(meters);
    TEST_ASSERT_EQUAL_INT32(-422000, mm);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, meters, arpt_mm_to_meters(mm));
}

/* ── Level of Detail ────────────────────────────────────────────────────── */

void test_geometric_error(void) {
    double root = 50000.0;
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 50000.0, arpt_geometric_error(root, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 25000.0, arpt_geometric_error(root, 1));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 12500.0, arpt_geometric_error(root, 2));
    TEST_ASSERT_DOUBLE_WITHIN(0.1,  1562.5,  arpt_geometric_error(root, 5));
}

void test_screen_space_error(void) {
    /* At distance=1000m, geom_error=100m, 1080p, 60° fov:
       sse = (100 * 1080) / (2 * 1000 * tan(30°)) = 108000 / 1154.7 ≈ 93.5 */
    double sse = arpt_screen_space_error(100.0, 1080.0, 1000.0, M_PI / 3.0);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 93.5, sse);
}

int main(void) {
    UNITY_BEGIN();
    /* Low-level quantization */
    RUN_TEST(test_dequantize_tile_origin);
    RUN_TEST(test_dequantize_tile_end);
    RUN_TEST(test_quantize_roundtrip);
    RUN_TEST(test_quantize_clamps_low);
    RUN_TEST(test_quantize_clamps_high);
    /* Tile bounds */
    RUN_TEST(test_tile_bounds_root_west);
    RUN_TEST(test_tile_bounds_root_east);
    RUN_TEST(test_tile_bounds_level5);
    /* Geodetic quantization */
    RUN_TEST(test_geodetic_quantize_lon_roundtrip);
    RUN_TEST(test_geodetic_quantize_lat_roundtrip);
    RUN_TEST(test_geodetic_quantize_tile_origin);
    /* Elevation */
    RUN_TEST(test_elevation_roundtrip);
    RUN_TEST(test_elevation_negative);
    /* LOD */
    RUN_TEST(test_geometric_error);
    RUN_TEST(test_screen_space_error);
    return UNITY_END();
}
