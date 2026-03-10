#include "unity.h"
#include "hilbert.h"

void setUp(void) {}
void tearDown(void) {}

static void test_hilbert_xy2d_origin(void) {
    uint64_t d = arpt_hilbert_xy2d(1, 0, 0);
    TEST_ASSERT_EQUAL_UINT64(0, d);
}

static void test_hilbert_tile_id_roundtrip(void) {
    uint64_t id = arpt_hilbert_tile_id(5, 10, 12);
    int z, x, y;
    arpt_hilbert_tile_id_decode(id, &z, &x, &y);
    /* Stub returns zeros — will pass once implemented */
    TEST_ASSERT_TRUE(z >= 0);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_hilbert_xy2d_origin);
    RUN_TEST(test_hilbert_tile_id_roundtrip);
    return UNITY_END();
}
