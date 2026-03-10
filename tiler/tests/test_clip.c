#include "unity.h"
#include "clip.h"

void setUp(void) {}
void tearDown(void) {}

static void test_assign_tiles_null(void) {
    /* Should not crash with NULL geometry. */
    arpt_assign_tiles(NULL, 0, NULL, NULL);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_assign_tiles_null);
    return UNITY_END();
}
