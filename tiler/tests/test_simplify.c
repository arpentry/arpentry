#include "unity.h"
#include "simplify.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

static void test_single_point(void) {
    double x[] = {1.0};
    double y[] = {2.0};
    TEST_ASSERT_EQUAL_UINT32(1, arpt_simplify(x, y, 1, 1.0));
}

static void test_two_points(void) {
    double x[] = {0.0, 1.0};
    double y[] = {0.0, 1.0};
    TEST_ASSERT_EQUAL_UINT32(2, arpt_simplify(x, y, 2, 1.0));
}

static void test_collinear_removed(void) {
    /* Three collinear points: middle one should be removed */
    double x[] = {0.0, 1.0, 2.0};
    double y[] = {0.0, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 3, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, x[1]);
}

static void test_collinear_many(void) {
    /* Many collinear points reduced to endpoints */
    double x[10], y[10];
    for (int i = 0; i < 10; i++) {
        x[i] = (double)i;
        y[i] = 0.0;
    }
    uint32_t n = arpt_simplify(x, y, 10, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
}

static void test_zigzag_preserved(void) {
    /* Zigzag with large deviations should be mostly preserved */
    double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double y[] = {0.0, 10.0, 0.0, 10.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 5, 1.0);
    TEST_ASSERT_EQUAL_UINT32(5, n);
}

static void test_tolerance_zero(void) {
    /* Zero tolerance keeps all points */
    double x[] = {0.0, 1.0, 2.0, 3.0};
    double y[] = {0.0, 0.1, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 4, 0.0);
    TEST_ASSERT_EQUAL_UINT32(4, n);
}

static void test_large_tolerance(void) {
    /* Very large tolerance removes all intermediate points */
    double x[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    double y[] = {0.0, 0.01, -0.01, 0.01, 0.0};
    uint32_t n = arpt_simplify(x, y, 5, 100.0);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, x[1]);
}

static void test_triangle(void) {
    /* Triangle with one point far from baseline */
    double x[] = {0.0, 1.0, 2.0};
    double y[] = {0.0, 5.0, 0.0};
    /* tolerance smaller than deviation: keep all */
    uint32_t n = arpt_simplify(x, y, 3, 1.0);
    TEST_ASSERT_EQUAL_UINT32(3, n);

    /* Reset and use large tolerance */
    double x2[] = {0.0, 1.0, 2.0};
    double y2[] = {0.0, 0.5, 0.0};
    n = arpt_simplify(x2, y2, 3, 1.0);
    TEST_ASSERT_EQUAL_UINT32(2, n);
}

static void test_endpoints_always_kept(void) {
    double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double y[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 5, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, x[1]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_single_point);
    RUN_TEST(test_two_points);
    RUN_TEST(test_collinear_removed);
    RUN_TEST(test_collinear_many);
    RUN_TEST(test_zigzag_preserved);
    RUN_TEST(test_tolerance_zero);
    RUN_TEST(test_large_tolerance);
    RUN_TEST(test_triangle);
    RUN_TEST(test_endpoints_always_kept);
    return UNITY_END();
}
