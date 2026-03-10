#include "unity.h"
#include "wkb.h"

void setUp(void) {}
void tearDown(void) {}

static void test_wkb_parse_null(void) {
    arpt_geom g = {0};
    bool ok = arpt_wkb_parse(NULL, 0, &g);
    TEST_ASSERT_FALSE(ok);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_wkb_parse_null);
    return UNITY_END();
}
