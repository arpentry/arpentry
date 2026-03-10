#include "unity.h"
#include "hilbert.h"

void setUp(void) {}
void tearDown(void) {}

/* ---- xy2d / d2xy round-trip ---- */

static void test_xy2d_origin(void) {
    TEST_ASSERT_EQUAL_UINT64(0, arpt_hilbert_xy2d(1, 0, 0));
}

static void test_xy2d_order1(void) {
    /* Order 1: 2×2 grid. Hilbert curve visits (0,0)=0 (1,0)=1 (1,1)=2 (0,1)=3 */
    /* Note: the exact order depends on convention; verify round-trip. */
    uint64_t d00 = arpt_hilbert_xy2d(1, 0, 0);
    uint64_t d10 = arpt_hilbert_xy2d(1, 1, 0);
    uint64_t d11 = arpt_hilbert_xy2d(1, 1, 1);
    uint64_t d01 = arpt_hilbert_xy2d(1, 0, 1);

    /* All four values must be distinct and in [0..3] */
    TEST_ASSERT_TRUE(d00 < 4);
    TEST_ASSERT_TRUE(d10 < 4);
    TEST_ASSERT_TRUE(d11 < 4);
    TEST_ASSERT_TRUE(d01 < 4);
    TEST_ASSERT_TRUE(d00 != d10 && d00 != d11 && d00 != d01);
    TEST_ASSERT_TRUE(d10 != d11 && d10 != d01);
    TEST_ASSERT_TRUE(d11 != d01);
}

static void test_roundtrip_order2(void) {
    /* Round-trip all 16 cells of a 4×4 grid */
    for (uint32_t x = 0; x < 4; x++) {
        for (uint32_t y = 0; y < 4; y++) {
            uint64_t d = arpt_hilbert_xy2d(2, x, y);
            uint32_t rx, ry;
            arpt_hilbert_d2xy(2, d, &rx, &ry);
            TEST_ASSERT_EQUAL_UINT32(x, rx);
            TEST_ASSERT_EQUAL_UINT32(y, ry);
        }
    }
}

static void test_roundtrip_order4(void) {
    /* Round-trip all 256 cells of a 16×16 grid */
    for (uint32_t x = 0; x < 16; x++) {
        for (uint32_t y = 0; y < 16; y++) {
            uint64_t d = arpt_hilbert_xy2d(4, x, y);
            uint32_t rx, ry;
            arpt_hilbert_d2xy(4, d, &rx, &ry);
            TEST_ASSERT_EQUAL_UINT32(x, rx);
            TEST_ASSERT_EQUAL_UINT32(y, ry);
        }
    }
}

static void test_d2xy_roundtrip_sequential(void) {
    /* Forward: d → (x,y) → d' should match */
    for (uint64_t d = 0; d < 64; d++) {
        uint32_t x, y;
        arpt_hilbert_d2xy(3, d, &x, &y);
        uint64_t d2 = arpt_hilbert_xy2d(3, x, y);
        TEST_ASSERT_EQUAL_UINT64(d, d2);
    }
}

static void test_bijective(void) {
    /* xy2d must be bijective (no collisions) for order 3 (8×8 = 64 cells) */
    uint64_t seen[64] = {0};
    for (uint32_t x = 0; x < 8; x++) {
        for (uint32_t y = 0; y < 8; y++) {
            uint64_t d = arpt_hilbert_xy2d(3, x, y);
            TEST_ASSERT_TRUE(d < 64);
            TEST_ASSERT_EQUAL_UINT64(0, seen[d]);
            seen[d] = 1;
        }
    }
}

/* ---- Tile ID encode/decode ---- */

static void test_tile_id_roundtrip_z0(void) {
    uint64_t id = arpt_hilbert_tile_id(0, 0, 0);
    int z, x, y;
    arpt_hilbert_tile_id_decode(id, &z, &x, &y);
    TEST_ASSERT_EQUAL_INT(0, z);
    TEST_ASSERT_EQUAL_INT(0, x);
    TEST_ASSERT_EQUAL_INT(0, y);
}

static void test_tile_id_roundtrip_z5(void) {
    uint64_t id = arpt_hilbert_tile_id(5, 10, 12);
    int z, x, y;
    arpt_hilbert_tile_id_decode(id, &z, &x, &y);
    TEST_ASSERT_EQUAL_INT(5, z);
    TEST_ASSERT_EQUAL_INT(10, x);
    TEST_ASSERT_EQUAL_INT(12, y);
}

static void test_tile_id_roundtrip_z14(void) {
    /* z=14: grid is 2^15 cols × 2^14 rows */
    int tz = 14, tx = 16000, ty = 8000;
    uint64_t id = arpt_hilbert_tile_id(tz, tx, ty);
    int z, x, y;
    arpt_hilbert_tile_id_decode(id, &z, &x, &y);
    TEST_ASSERT_EQUAL_INT(tz, z);
    TEST_ASSERT_EQUAL_INT(tx, x);
    TEST_ASSERT_EQUAL_INT(ty, y);
}

static void test_tile_id_zoom_ordering(void) {
    /* IDs at different zoom levels: higher zoom → higher tile_id
       because zoom is in the top bits */
    uint64_t id_z0 = arpt_hilbert_tile_id(0, 0, 0);
    uint64_t id_z1 = arpt_hilbert_tile_id(1, 0, 0);
    uint64_t id_z5 = arpt_hilbert_tile_id(5, 0, 0);
    TEST_ASSERT_TRUE(id_z0 < id_z1);
    TEST_ASSERT_TRUE(id_z1 < id_z5);
}

static void test_tile_id_spatial_locality(void) {
    /* Adjacent tiles at same zoom should have close Hilbert IDs */
    uint64_t id_a = arpt_hilbert_tile_id(4, 5, 5);
    uint64_t id_b = arpt_hilbert_tile_id(4, 5, 6);
    uint64_t id_far = arpt_hilbert_tile_id(4, 0, 15);
    /* a and b should be closer than a and far */
    uint64_t diff_near = id_a > id_b ? id_a - id_b : id_b - id_a;
    uint64_t diff_far  = id_a > id_far ? id_a - id_far : id_far - id_a;
    TEST_ASSERT_TRUE(diff_near < diff_far);
}

static void test_tile_id_null_pointers(void) {
    /* Should not crash with NULL output pointers */
    arpt_hilbert_tile_id_decode(0, NULL, NULL, NULL);
    arpt_hilbert_d2xy(1, 0, NULL, NULL);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_xy2d_origin);
    RUN_TEST(test_xy2d_order1);
    RUN_TEST(test_roundtrip_order2);
    RUN_TEST(test_roundtrip_order4);
    RUN_TEST(test_d2xy_roundtrip_sequential);
    RUN_TEST(test_bijective);
    RUN_TEST(test_tile_id_roundtrip_z0);
    RUN_TEST(test_tile_id_roundtrip_z5);
    RUN_TEST(test_tile_id_roundtrip_z14);
    RUN_TEST(test_tile_id_zoom_ordering);
    RUN_TEST(test_tile_id_spatial_locality);
    RUN_TEST(test_tile_id_null_pointers);
    return UNITY_END();
}
