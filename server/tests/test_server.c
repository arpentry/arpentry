#include "unity.h"
#include "tile_path.h"

void setUp(void) {}
void tearDown(void) {}

/* Valid paths */

void test_parse_level0(void) {
    int level, x, y;
    TEST_ASSERT_TRUE(arpt_parse_tile_path("/0/0/0.arpt", &level, &x, &y));
    TEST_ASSERT_EQUAL_INT(0, level);
    TEST_ASSERT_EQUAL_INT(0, x);
    TEST_ASSERT_EQUAL_INT(0, y);
}

void test_parse_level0_max_x(void) {
    /* Level 0: x in [0, 1], y in [0, 0] */
    int level, x, y;
    TEST_ASSERT_TRUE(arpt_parse_tile_path("/0/1/0.arpt", &level, &x, &y));
    TEST_ASSERT_EQUAL_INT(0, level);
    TEST_ASSERT_EQUAL_INT(1, x);
    TEST_ASSERT_EQUAL_INT(0, y);
}

void test_parse_mid_level(void) {
    int level, x, y;
    TEST_ASSERT_TRUE(arpt_parse_tile_path("/5/32/16.arpt", &level, &x, &y));
    TEST_ASSERT_EQUAL_INT(5, level);
    TEST_ASSERT_EQUAL_INT(32, x);
    TEST_ASSERT_EQUAL_INT(16, y);
}

void test_parse_max_level(void) {
    /* Level 21: x in [0, 4194303], y in [0, 2097151] */
    int level, x, y;
    TEST_ASSERT_TRUE(arpt_parse_tile_path("/21/4194303/2097151.arpt",
                                          &level, &x, &y));
    TEST_ASSERT_EQUAL_INT(21, level);
    TEST_ASSERT_EQUAL_INT(4194303, x);
    TEST_ASSERT_EQUAL_INT(2097151, y);
}

/* Invalid paths */

void test_reject_wrong_extension(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/0/0/0.pbf", &level, &x, &y));
}

void test_reject_no_extension(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/0/0/0", &level, &x, &y));
}

void test_reject_missing_components(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/0/0.arpt", &level, &x, &y));
}

void test_reject_negative_level(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/-1/0/0.arpt", &level, &x, &y));
}

void test_reject_level_too_high(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/22/0/0.arpt", &level, &x, &y));
}

void test_reject_x_out_of_range(void) {
    /* Level 0: max x = 1 */
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/0/2/0.arpt", &level, &x, &y));
}

void test_reject_y_out_of_range(void) {
    /* Level 0: max y = 0 */
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/0/0/1.arpt", &level, &x, &y));
}

void test_reject_null_uri(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path(NULL, &level, &x, &y));
}

void test_reject_empty_uri(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("", &level, &x, &y));
}

void test_reject_tileset_json(void) {
    int level, x, y;
    TEST_ASSERT_FALSE(arpt_parse_tile_path("/tileset.json", &level, &x, &y));
}

int main(void) {
    UNITY_BEGIN();
    /* Valid paths */
    RUN_TEST(test_parse_level0);
    RUN_TEST(test_parse_level0_max_x);
    RUN_TEST(test_parse_mid_level);
    RUN_TEST(test_parse_max_level);
    /* Invalid paths */
    RUN_TEST(test_reject_wrong_extension);
    RUN_TEST(test_reject_no_extension);
    RUN_TEST(test_reject_missing_components);
    RUN_TEST(test_reject_negative_level);
    RUN_TEST(test_reject_level_too_high);
    RUN_TEST(test_reject_x_out_of_range);
    RUN_TEST(test_reject_y_out_of_range);
    RUN_TEST(test_reject_null_uri);
    RUN_TEST(test_reject_empty_uri);
    RUN_TEST(test_reject_tileset_json);
    return UNITY_END();
}
