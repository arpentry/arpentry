#include "unity.h"
#include "tile_build.h"

void setUp(void) {}
void tearDown(void) {}

static void test_tile_builder_create_free(void) {
    arpt_bounds b = {0.0, 0.0, 1.0, 1.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    /* Stub returns NULL — just verify no crash. */
    arpt_tile_builder_free(tb);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_tile_builder_create_free);
    return UNITY_END();
}
