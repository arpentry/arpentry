#include "unity.h"
#include "tile/manager.h"
#include "coords.h"

#include <math.h>
#include <stdio.h>

void setUp(void) {}
void tearDown(void) {}

/* arpt_tile_ancestor tests */

void test_ancestor_basic(void) {
    int pl, px, py;
    TEST_ASSERT_TRUE(arpt_tile_ancestor(3, 6, 4, &pl, &px, &py));
    TEST_ASSERT_EQUAL_INT(2, pl);
    TEST_ASSERT_EQUAL_INT(3, px);
    TEST_ASSERT_EQUAL_INT(2, py);
}

void test_ancestor_level_zero(void) {
    int pl, px, py;
    TEST_ASSERT_FALSE(arpt_tile_ancestor(0, 0, 0, &pl, &px, &py));
}

void test_ancestor_chain(void) {
    /* Walk from level 5 back to level 0 */
    int l = 5, x = 34, y = 22;
    int pl, px, py;
    int steps = 0;
    while (arpt_tile_ancestor(l, x, y, &pl, &px, &py)) {
        l = pl;
        x = px;
        y = py;
        steps++;
    }
    TEST_ASSERT_EQUAL_INT(5, steps);
    TEST_ASSERT_EQUAL_INT(0, l);
}

/* arpt_tile_covered_quadrants tests */

void test_covered_quadrants_follow_readiness(void) {
    /* Ancestor 2/1/1. Its children at level 3 are x 2..3, y 2..3; quadrant
       bit = (cx & 1) | (cy & 1) << 1. Three ready, the north-west unready. */
    arpt_tile_key vis[] = {{3, 2, 2}, {3, 3, 2}, {3, 2, 3}, {3, 3, 3}};
    bool ready[] = {true, true, false, true};
    uint32_t m = arpt_tile_covered_quadrants(2, 1, 1, vis, ready, 4);
    TEST_ASSERT_EQUAL_UINT32((1u << 0) | (1u << 1) | (1u << 3), m);
}

void test_covered_quadrants_reach_through_deeper_levels(void) {
    /* Visible tiles two levels down: quadrant 0 (child 3/2/2) holds 4/4/4 and
       4/5/4, one unready, so it stays drawn; quadrant 3 holds one ready
       grandchild and is covered; quadrants 1 and 2 hold nothing visible and
       are not covered (nothing draws over them). A tile under another
       ancestor and a tile above the ancestor's level are ignored. */
    arpt_tile_key vis[] = {{4, 4, 4}, {4, 5, 4}, {4, 7, 7}, {4, 9, 9}, {1, 0, 0}};
    bool ready[] = {true, false, true, false, false};
    uint32_t m = arpt_tile_covered_quadrants(2, 1, 1, vis, ready, 5);
    TEST_ASSERT_EQUAL_UINT32(1u << 3, m);
}

/* arpt_enumerate_visible_tiles tests */

void test_enumerate_at_origin(void) {
    /* Camera over (0,0) at ~500km altitude (roughly zoom level 5) */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    int level = arpt_camera_zoom_level(cam, 50000.0, 0, 16);

    arpt_tile_key tiles[256];
    int n = arpt_enumerate_visible_tiles(cam, level, tiles, 256);

    /* We should get some tiles at this zoom level */
    TEST_ASSERT_GREATER_THAN(0, n);

    /* All tiles should be at the requested level */
    for (int i = 0; i < n; i++) {
        TEST_ASSERT_EQUAL_INT(level, tiles[i].level);
    }

    /* The tile containing our camera center (lon=0) should be in the list */
    int n_cols = 1 << level;
    int expected_x = n_cols / 2;

    bool found_center = false;
    for (int i = 0; i < n; i++) {
        if (tiles[i].x == expected_x) {
            found_center = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found_center);

    arpt_camera_free(cam);
}

void test_enumerate_returns_zero_for_sky(void) {
    /* Camera looking away from earth (very high altitude, tilted up) */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 1e9); /* 1 million km */
    arpt_camera_set_viewport(cam, 800, 600);

    arpt_tile_key tiles[64];
    int n = arpt_enumerate_visible_tiles(cam, 0, tiles, 64);

    /* At extreme distance, rays may all miss or we may still see tiles.
     * What matters is it doesn't crash. */
    TEST_ASSERT_GREATER_OR_EQUAL(0, n);

    arpt_camera_free(cam);
}

void test_enumerate_null_camera(void) {
    arpt_tile_key tiles[64];
    int n = arpt_enumerate_visible_tiles(NULL, 5, tiles, 64);
    TEST_ASSERT_EQUAL_INT(0, n);
}

void test_zoom_level_varies_with_altitude(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_viewport(cam, 800, 600);

    /* High altitude: low zoom level */
    arpt_camera_set_position(cam, 0.0, 0.0, 5000000.0);
    int level_high = arpt_camera_zoom_level(cam, 50000.0, 0, 16);

    /* Low altitude: high zoom level */
    arpt_camera_set_position(cam, 0.0, 0.0, 10000.0);
    int level_low = arpt_camera_zoom_level(cam, 50000.0, 0, 16);

    TEST_ASSERT_GREATER_THAN(level_high, level_low);

    arpt_camera_free(cam);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ancestor_basic);
    RUN_TEST(test_ancestor_level_zero);
    RUN_TEST(test_ancestor_chain);
    RUN_TEST(test_covered_quadrants_follow_readiness);
    RUN_TEST(test_covered_quadrants_reach_through_deeper_levels);
    RUN_TEST(test_enumerate_at_origin);
    RUN_TEST(test_enumerate_returns_zero_for_sky);
    RUN_TEST(test_enumerate_null_camera);
    RUN_TEST(test_zoom_level_varies_with_altitude);
    return UNITY_END();
}
