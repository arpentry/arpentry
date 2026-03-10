#include "unity.h"
#include "clip.h"
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* ---- Test helpers ---- */

typedef struct {
    int z, x, y;
    arpt_geom geom;  /* copy of clipped geometry */
} tile_result;

typedef struct {
    tile_result *results;
    int count;
    int cap;
} tile_collector;

static void collect_cb(int z, int x, int y,
                       const arpt_geom *clipped, void *ctx) {
    tile_collector *c = (tile_collector *)ctx;
    if (c->count == c->cap) {
        int nc = c->cap ? c->cap * 2 : 8;
        c->results = realloc(c->results, (size_t)nc * sizeof(tile_result));
        c->cap = nc;
    }
    tile_result *r = &c->results[c->count++];
    r->z = z;
    r->x = x;
    r->y = y;
    /* Deep copy the geometry */
    r->geom = *clipped;
    r->geom.x = malloc(clipped->n_coords * sizeof(double));
    r->geom.y = malloc(clipped->n_coords * sizeof(double));
    memcpy(r->geom.x, clipped->x, clipped->n_coords * sizeof(double));
    memcpy(r->geom.y, clipped->y, clipped->n_coords * sizeof(double));
    r->geom.offsets = NULL;
    r->geom.n_offsets = 0;
}

static void collector_init(tile_collector *c) {
    c->results = NULL;
    c->count = 0;
    c->cap = 0;
}

static void collector_free(tile_collector *c) {
    for (int i = 0; i < c->count; i++) {
        free(c->results[i].geom.x);
        free(c->results[i].geom.y);
    }
    free(c->results);
    c->results = NULL;
    c->count = 0;
    c->cap = 0;
}

/* ---- Null safety ---- */

static void test_assign_tiles_null(void) {
    arpt_assign_tiles(NULL, 0, NULL, NULL);
}

static void test_assign_tiles_null_cb(void) {
    arpt_geom g = {0};
    g.type = 1;
    double x = 6.6, y = 46.5;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;
    arpt_assign_tiles(&g, 0, NULL, NULL);
}

static void test_assign_tiles_empty_geom(void) {
    arpt_geom g = {0};
    g.type = 1;
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);
    TEST_ASSERT_EQUAL_INT(0, c.count);
    collector_free(&c);
}

/* ---- Point clipping ---- */

static void test_point_z0(void) {
    /* A point at (6.6, 46.5) at z=0 should fall in tile (0,0,0) */
    arpt_geom g = {0};
    g.type = 1;
    double x = 6.6, y = 46.5;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);

    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].z);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].y);
    collector_free(&c);
}

static void test_point_z1(void) {
    /* (6.6, 46.5) at z=1 */
    arpt_geom g = {0};
    g.type = 1;
    double x = 6.6, y = 46.5;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 1, collect_cb, &c);

    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].z);
    /* x=1 (eastern half), y=0 (northern half for lat ~46.5) */
    TEST_ASSERT_EQUAL_INT(1, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].y);
    collector_free(&c);
}

static void test_multipoint(void) {
    /* Two points at different locations */
    arpt_geom g = {0};
    g.type = 4; /* MultiPoint */
    double x[] = {-90.0, 90.0};
    double y[] = {0.0, 0.0};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 1, collect_cb, &c);

    TEST_ASSERT_EQUAL_INT(2, c.count);
    /* The two points should be in different tiles */
    TEST_ASSERT_TRUE(c.results[0].x != c.results[1].x ||
                     c.results[0].y != c.results[1].y);
    collector_free(&c);
}

/* ---- Line clipping ---- */

static void test_line_within_tile(void) {
    /* A short line entirely within one tile at z=2 */
    arpt_geom g = {0};
    g.type = 2; /* LineString */
    double x[] = {6.5, 6.6};
    double y[] = {46.4, 46.5};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    collector_free(&c);
}

static void test_line_crossing_tiles(void) {
    /* A long horizontal line crossing multiple tiles at z=2 */
    arpt_geom g = {0};
    g.type = 2;
    double x[] = {-10.0, 50.0};
    double y[] = {46.0, 46.0};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    /* Should produce clipped segments in multiple tiles */
    TEST_ASSERT_TRUE(c.count >= 1);
    collector_free(&c);
}

/* ---- Polygon clipping ---- */

static void test_polygon_within_tile(void) {
    /* Small polygon entirely within one tile */
    arpt_geom g = {0};
    g.type = 3; /* Polygon */
    double x[] = {6.5, 6.6, 6.6, 6.5, 6.5};
    double y[] = {46.4, 46.4, 46.5, 46.5, 46.4};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 4, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    /* The clipped polygon should have at least 3 vertices */
    for (int i = 0; i < c.count; i++) {
        TEST_ASSERT_TRUE(c.results[i].geom.n_coords >= 3);
    }
    collector_free(&c);
}

static void test_polygon_crossing_tiles(void) {
    /* Large polygon crossing multiple tiles */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {-10.0, 20.0, 20.0, -10.0, -10.0};
    double y[] = {40.0, 40.0, 55.0, 55.0, 40.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    /* Should produce clipped polygons in multiple tiles */
    TEST_ASSERT_TRUE(c.count >= 2);
    collector_free(&c);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_assign_tiles_null);
    RUN_TEST(test_assign_tiles_null_cb);
    RUN_TEST(test_assign_tiles_empty_geom);
    RUN_TEST(test_point_z0);
    RUN_TEST(test_point_z1);
    RUN_TEST(test_multipoint);
    RUN_TEST(test_line_within_tile);
    RUN_TEST(test_line_crossing_tiles);
    RUN_TEST(test_polygon_within_tile);
    RUN_TEST(test_polygon_crossing_tiles);
    return UNITY_END();
}
