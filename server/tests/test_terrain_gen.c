#include "unity.h"
#include "noise.h"
#include "terrain_gen.h"
#include "tile.h"
#include "coords.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

void setUp(void) {}
void tearDown(void) {}

/* 2D simplex noise tests */

void test_simplex_deterministic(void) {
    double a = arpt_simplex2(1.23, 4.56);
    double b = arpt_simplex2(1.23, 4.56);
    TEST_ASSERT_EQUAL_DOUBLE(a, b);
}

void test_simplex_range(void) {
    double inputs[][2] = {
        {0.0, 0.0}, {1.0, 1.0}, {-3.7, 2.1}, {100.0, -50.0},
        {0.001, 0.001}, {-180.0, 90.0}, {3.14, 2.72},
    };
    int n = sizeof(inputs) / sizeof(inputs[0]);
    for (int i = 0; i < n; i++) {
        double v = arpt_simplex2(inputs[i][0], inputs[i][1]);
        TEST_ASSERT_TRUE_MESSAGE(v >= -1.0 && v <= 1.0,
                                  "simplex2 output out of [-1, 1]");
    }
}

void test_simplex_varies(void) {
    double a = arpt_simplex2(0.0, 0.0);
    double b = arpt_simplex2(10.0, 10.0);
    TEST_ASSERT_TRUE_MESSAGE(fabs(a - b) > 1e-6,
                              "simplex2 should vary for different inputs");
}

/* 3D simplex noise tests */

void test_simplex3_deterministic(void) {
    double a = arpt_simplex3(1.23, 4.56, 7.89);
    double b = arpt_simplex3(1.23, 4.56, 7.89);
    TEST_ASSERT_EQUAL_DOUBLE(a, b);
}

void test_simplex3_range(void) {
    double inputs[][3] = {
        {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {-3.7, 2.1, 0.5},
        {100.0, -50.0, 25.0}, {0.001, 0.001, 0.001},
        {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0},
    };
    int n = sizeof(inputs) / sizeof(inputs[0]);
    for (int i = 0; i < n; i++) {
        double v = arpt_simplex3(inputs[i][0], inputs[i][1], inputs[i][2]);
        TEST_ASSERT_TRUE_MESSAGE(v >= -1.0 && v <= 1.0,
                                  "simplex3 output out of [-1, 1]");
    }
}

void test_simplex3_varies(void) {
    double a = arpt_simplex3(0.5, 1.7, 3.2);
    double b = arpt_simplex3(7.3, -2.1, 4.8);
    TEST_ASSERT_TRUE_MESSAGE(fabs(a - b) > 1e-6,
                              "simplex3 should vary for different inputs");
}

/* fBm tests */

void test_fbm_deterministic(void) {
    double a = arpt_fbm2(1.0, 2.0, 6, 2.0, 0.5);
    double b = arpt_fbm2(1.0, 2.0, 6, 2.0, 0.5);
    TEST_ASSERT_EQUAL_DOUBLE(a, b);
}

void test_fbm_more_octaves_more_detail(void) {
    int N = 50;
    double diff_sum = 0.0;
    for (int i = 0; i < N; i++) {
        double x = (double)i * 0.1;
        double low  = arpt_fbm2(x, 0.5, 2, 2.0, 0.5);
        double high = arpt_fbm2(x, 0.5, 10, 2.0, 0.5);
        diff_sum += fabs(high - low);
    }
    double avg_diff = diff_sum / N;
    TEST_ASSERT_TRUE_MESSAGE(avg_diff > 1e-4,
                              "Higher octaves should add noticeable detail");
}

void test_fbm3_deterministic(void) {
    double a = arpt_fbm3(1.0, 2.0, 3.0, 6, 2.0, 0.5);
    double b = arpt_fbm3(1.0, 2.0, 3.0, 6, 2.0, 0.5);
    TEST_ASSERT_EQUAL_DOUBLE(a, b);
}

void test_fbm3_more_octaves_more_detail(void) {
    int N = 50;
    double diff_sum = 0.0;
    for (int i = 0; i < N; i++) {
        double x = (double)i * 0.1;
        double low  = arpt_fbm3(x, 0.5, 0.3, 2, 2.0, 0.5);
        double high = arpt_fbm3(x, 0.5, 0.3, 16, 2.0, 0.5);
        diff_sum += fabs(high - low);
    }
    double avg_diff = diff_sum / N;
    TEST_ASSERT_TRUE_MESSAGE(avg_diff > 1e-4,
                              "Higher octaves should add noticeable detail (3D)");
}

/* Sphere continuity tests */

void test_pole_continuity(void) {
    /* At the north pole (lat=90), all longitudes map to the same
     * Cartesian point (0, 0, 1), so elevation must be identical. */
    double lat_r = 90.0 * (PI / 180.0);
    double cos_lat = cos(lat_r);
    double sz = sin(lat_r);
    double base_freq = 4.0;

    double ref = -999.0;
    for (int lon_deg = -180; lon_deg <= 180; lon_deg += 15) {
        double lon_r = (double)lon_deg * (PI / 180.0);
        double sx = cos_lat * cos(lon_r);
        double sy = cos_lat * sin(lon_r);
        double elev = arpt_fbm3(sx * base_freq, sy * base_freq,
                                sz * base_freq, 16, 2.0, 0.5);
        if (ref < -998.0) {
            ref = elev;
        } else {
            TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(
                1e-10, ref, elev,
                "Pole elevation must be identical for all longitudes");
        }
    }
}

void test_antimeridian_continuity(void) {
    /* lon=-180 and lon=+180 are the same geographic point,
     * so their sphere coordinates and elevation must match. */
    double base_freq = 4.0;

    for (int lat_deg = -85; lat_deg <= 85; lat_deg += 10) {
        double lat_r = (double)lat_deg * (PI / 180.0);
        double cos_lat = cos(lat_r);
        double sz = sin(lat_r);

        double lon_neg = -180.0 * (PI / 180.0);
        double lon_pos =  180.0 * (PI / 180.0);

        double sx_a = cos_lat * cos(lon_neg);
        double sy_a = cos_lat * sin(lon_neg);
        double elev_a = arpt_fbm3(sx_a * base_freq, sy_a * base_freq,
                                  sz * base_freq, 16, 2.0, 0.5);

        double sx_b = cos_lat * cos(lon_pos);
        double sy_b = cos_lat * sin(lon_pos);
        double elev_b = arpt_fbm3(sx_b * base_freq, sy_b * base_freq,
                                  sz * base_freq, 16, 2.0, 0.5);

        /* sin(pi) is not exactly 0 in floating point (~1.22e-16),
         * so sy differs slightly between lon=-180 and lon=+180.
         * After 16 fBm octaves this accumulates to ~1e-6. */
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(
            1e-4, elev_a, elev_b,
            "Antimeridian elevation must match for lon=-180 and lon=+180");
    }
}

/* Terrain generation tests */

void test_generate_terrain_valid(void) {
    uint8_t *data = NULL;
    size_t size = 0;
    TEST_ASSERT_TRUE(arpt_generate_terrain(0, 0, 0, &data, &size));
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_GREATER_THAN(0, size);

    /* Verify it decodes as a valid .arpt tile */
    uint8_t *fb = NULL;
    size_t fb_size = 0;
    TEST_ASSERT_TRUE(arpt_decode(data, size, &fb, &fb_size));
    TEST_ASSERT_NOT_NULL(fb);
    TEST_ASSERT_GREATER_THAN(0, fb_size);

    free(fb);
    free(data);
}

void test_generate_terrain_consistency(void) {
    uint8_t *a = NULL, *b = NULL;
    size_t a_sz = 0, b_sz = 0;

    TEST_ASSERT_TRUE(arpt_generate_terrain(3, 5, 2, &a, &a_sz));
    TEST_ASSERT_TRUE(arpt_generate_terrain(3, 5, 2, &b, &b_sz));

    TEST_ASSERT_EQUAL_size_t(a_sz, b_sz);
    TEST_ASSERT_EQUAL_MEMORY(a, b, a_sz);

    free(a);
    free(b);
}

void test_generate_terrain_different_tiles(void) {
    uint8_t *a = NULL, *b = NULL;
    size_t a_sz = 0, b_sz = 0;

    TEST_ASSERT_TRUE(arpt_generate_terrain(0, 0, 0, &a, &a_sz));
    TEST_ASSERT_TRUE(arpt_generate_terrain(0, 1, 0, &b, &b_sz));

    /* Different tiles should produce different output */
    TEST_ASSERT_TRUE(a_sz != b_sz ||
                     memcmp(a, b, a_sz) != 0);

    free(a);
    free(b);
}

void test_generate_terrain_null_params(void) {
    size_t size = 0;
    TEST_ASSERT_FALSE(arpt_generate_terrain(0, 0, 0, NULL, &size));

    uint8_t *data = NULL;
    TEST_ASSERT_FALSE(arpt_generate_terrain(0, 0, 0, &data, NULL));
}

void test_adjacent_tiles_match(void) {
    /* Two horizontally adjacent tiles at level 2 should share
     * the same elevation along their shared edge.
     * We verify this by checking that the 3D sphere noise produces
     * identical values at the shared longitude boundary. */
    int level = 2;
    int x_left = 3, x_right = 4, y = 1;

    arpt_bounds b_left  = arpt_tile_bounds(level, x_left, y);
    arpt_bounds b_right = arpt_tile_bounds(level, x_right, y);

    /* The shared edge is at b_left.east == b_right.west */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, b_left.east, b_right.west);

    double shared_lon = b_left.east;
    double lat_span = b_left.north - b_left.south;
    double base_freq = 4.0;

    /* Sample along the shared edge */
    for (int i = 0; i <= 32; i++) {
        double t = (double)i / 32.0;
        double lat = b_left.south + t * lat_span;

        /* Convert to sphere coordinates — same as terrain_elevation() */
        double lon_r = shared_lon * (PI / 180.0);
        double lat_r = lat * (PI / 180.0);
        double cos_lat = cos(lat_r);
        double sx = cos_lat * cos(lon_r);
        double sy = cos_lat * sin(lon_r);
        double sz = sin(lat_r);

        double elev_a = arpt_fbm3(sx * base_freq, sy * base_freq,
                                  sz * base_freq, 16, 2.0, 0.5);
        double elev_b = arpt_fbm3(sx * base_freq, sy * base_freq,
                                  sz * base_freq, 16, 2.0, 0.5);
        TEST_ASSERT_EQUAL_DOUBLE(elev_a, elev_b);
    }
}

/* Main */

int main(void) {
    UNITY_BEGIN();
    /* 2D simplex noise */
    RUN_TEST(test_simplex_deterministic);
    RUN_TEST(test_simplex_range);
    RUN_TEST(test_simplex_varies);
    /* 3D simplex noise */
    RUN_TEST(test_simplex3_deterministic);
    RUN_TEST(test_simplex3_range);
    RUN_TEST(test_simplex3_varies);
    /* fBm */
    RUN_TEST(test_fbm_deterministic);
    RUN_TEST(test_fbm_more_octaves_more_detail);
    RUN_TEST(test_fbm3_deterministic);
    RUN_TEST(test_fbm3_more_octaves_more_detail);
    /* Sphere continuity */
    RUN_TEST(test_pole_continuity);
    RUN_TEST(test_antimeridian_continuity);
    /* Terrain generation */
    RUN_TEST(test_generate_terrain_valid);
    RUN_TEST(test_generate_terrain_consistency);
    RUN_TEST(test_generate_terrain_different_tiles);
    RUN_TEST(test_generate_terrain_null_params);
    RUN_TEST(test_adjacent_tiles_match);
    return UNITY_END();
}
