#include "unity.h"
#include "simplify.h"

void setUp(void) {}
void tearDown(void) {}

static void test_simplify_noop(void) {
    double x[] = {0.0, 1.0, 2.0};
    double y[] = {0.0, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 3, 1.0);
    TEST_ASSERT_EQUAL_UINT32(3, n);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_simplify_noop);
    return UNITY_END();
}
