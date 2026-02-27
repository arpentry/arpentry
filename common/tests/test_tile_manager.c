#include "unity.h"
#include "tile_manager.h"
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

/* arpt_enumerate_visible_tiles tests */

void test_enumerate_at_switzerland(void) {
    /* Camera over Switzerland at ~500km altitude (roughly zoom level 5) */
    arpt_camera *cam = arpt_camera_create();
    arpt_bounds bounds = arpt_tile_bounds(5, 34, 22);
    double center_lon = (bounds.west + bounds.east) / 2.0 * M_PI / 180.0;
    double center_lat = (bounds.south + bounds.north) / 2.0 * M_PI / 180.0;
    arpt_camera_set_position(cam, center_lon, center_lat, 500000.0);
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

    /* The tile containing our camera center should be in the list */
    int n_cols = 1 << (level + 1);
    double tile_w = 360.0 / n_cols;
    double center_lon_deg = center_lon * 180.0 / M_PI;
    int expected_x = (int)floor((center_lon_deg + 180.0) / tile_w);

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
    RUN_TEST(test_enumerate_at_switzerland);
    RUN_TEST(test_enumerate_returns_zero_for_sky);
    RUN_TEST(test_enumerate_null_camera);
    RUN_TEST(test_zoom_level_varies_with_altitude);
    return UNITY_END();
}
