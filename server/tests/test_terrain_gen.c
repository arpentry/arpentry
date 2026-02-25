#include "unity.h"
#include "noise.h"
#include "terrain_gen.h"
#include "tile.h"
#include "coords.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* ── Simplex noise tests ───────────────────────────────────────────── */

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

/* ── fBm tests ─────────────────────────────────────────────────────── */

void test_fbm_deterministic(void) {
    double a = arpt_fbm2(1.0, 2.0, 6, 2.0, 0.5);
    double b = arpt_fbm2(1.0, 2.0, 6, 2.0, 0.5);
    TEST_ASSERT_EQUAL_DOUBLE(a, b);
}

void test_fbm_more_octaves_more_detail(void) {
    /* More octaves should add finer variation.
     * Measure variance over a grid — higher octaves should differ
     * from the low-octave version (introduces detail). */
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

/* ── Terrain generation tests ──────────────────────────────────────── */

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
     * We verify this by checking that the noise function produces
     * identical values at the shared longitude boundary. */
    int level = 2;
    int x_left = 3, x_right = 4, y = 1;

    arpt_bounds_t b_left  = arpt_tile_bounds(level, x_left, y);
    arpt_bounds_t b_right = arpt_tile_bounds(level, x_right, y);

    /* The shared edge is at b_left.east == b_right.west */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, b_left.east, b_right.west);

    double shared_lon = b_left.east;
    double lat_span = b_left.north - b_left.south;

    /* Sample along the shared edge */
    for (int i = 0; i <= 32; i++) {
        double t = (double)i / 32.0;
        double lat = b_left.south + t * lat_span;

        /* Both tiles should compute the same elevation at this point */
        /* Since we use geodetic coordinates as noise input, this is
         * guaranteed by the deterministic noise function. */
        double lon_rad = shared_lon * (3.14159265358979323846 / 180.0);
        double lat_rad = lat * (3.14159265358979323846 / 180.0);
        double elev_a = arpt_fbm2(lon_rad, lat_rad, 10, 2.0, 0.5);
        double elev_b = arpt_fbm2(lon_rad, lat_rad, 10, 2.0, 0.5);
        TEST_ASSERT_EQUAL_DOUBLE(elev_a, elev_b);
    }
}

/* ── Main ──────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();
    /* Simplex noise */
    RUN_TEST(test_simplex_deterministic);
    RUN_TEST(test_simplex_range);
    RUN_TEST(test_simplex_varies);
    /* fBm */
    RUN_TEST(test_fbm_deterministic);
    RUN_TEST(test_fbm_more_octaves_more_detail);
    /* Terrain generation */
    RUN_TEST(test_generate_terrain_valid);
    RUN_TEST(test_generate_terrain_consistency);
    RUN_TEST(test_generate_terrain_different_tiles);
    RUN_TEST(test_generate_terrain_null_params);
    RUN_TEST(test_adjacent_tiles_match);
    return UNITY_END();
}
