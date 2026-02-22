#include "unity.h"
#include "arpentry/common.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_dequantize_tile_origin);
    RUN_TEST(test_dequantize_tile_end);
    RUN_TEST(test_quantize_roundtrip);
    RUN_TEST(test_quantize_clamps_low);
    RUN_TEST(test_quantize_clamps_high);
    return UNITY_END();
}
