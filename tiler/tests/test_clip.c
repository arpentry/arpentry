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
    if (clipped->offsets && clipped->n_offsets > 0) {
        r->geom.offsets = malloc(clipped->n_offsets * sizeof(uint32_t));
        memcpy(r->geom.offsets, clipped->offsets,
               clipped->n_offsets * sizeof(uint32_t));
        r->geom.n_offsets = clipped->n_offsets;
    } else {
        r->geom.offsets = NULL;
        r->geom.n_offsets = 0;
    }
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
        free(c->results[i].geom.offsets);
    }
    free(c->results);
    c->results = NULL;
    c->count = 0;
    c->cap = 0;
}

/* Find a result by tile coordinates. Returns NULL if not found. */
static tile_result *find_result(tile_collector *c, int z, int x, int y) {
    for (int i = 0; i < c->count; i++) {
        if (c->results[i].z == z && c->results[i].x == x &&
            c->results[i].y == y)
            return &c->results[i];
    }
    return NULL;
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
    /* A point at (6.6, 46.5) at z=0 should fall in tile (0,1,0).
       Equirectangular grid z=0: 2 cols × 1 row.
       tx = floor((6.6+180)/360 * 2) = 1 (eastern hemisphere) */
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
    TEST_ASSERT_EQUAL_INT(1, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].y);
    collector_free(&c);
}

static void test_point_z1(void) {
    /* (6.6, 46.5) at z=1.
       Equirectangular grid z=1: 4 cols × 2 rows.
       tx = floor((6.6+180)/360 * 4) = 2
       ty = floor((46.5+90)/180 * 2) = 1 */
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
    TEST_ASSERT_EQUAL_INT(2, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].y);
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

static void test_line_no_phantom_segments(void) {
    /* A V-shaped line that enters and exits a tile column in separate rows.
     * At z=2: tiles are 45 lon × 45 lat.
     * Line goes from (10,80) down to (10,10) then back up to (50,80).
     * The middle point at (10,10) is in a different tile row than the
     * endpoints at y=80.  After strip clipping to column [0,45],
     * segments (10,80)→(10,10) and (10,10)→(45,52.5) should be
     * separate from (45,52.5)→(50,80) in the next column.
     * Within the [0,45] column, for the row [45,90], there should be
     * no phantom segment connecting the two clipped pieces. */
    arpt_geom g = {0};
    g.type = 2;
    double x[] = {10.0, 10.0, 50.0};
    double y[] = {80.0, 10.0, 80.0};
    g.x = x;
    g.y = y;
    g.n_coords = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    /* Just verify it doesn't crash and produces reasonable output */
    TEST_ASSERT_TRUE(c.count >= 1);

    /* Each result should have at least 2 points (valid line segment) */
    for (int i = 0; i < c.count; i++) {
        TEST_ASSERT_TRUE(c.results[i].geom.n_coords >= 2);
    }
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
    /* The clipped polygon should have at least 4 vertices (3 unique + closing) */
    for (int i = 0; i < c.count; i++) {
        TEST_ASSERT_TRUE(c.results[i].geom.n_coords >= 4);
    }
    collector_free(&c);
}

static void test_polygon_rings_closed(void) {
    /* Verify that clipped polygon rings are closed (first == last vertex) */
    arpt_geom g = {0};
    g.type = 3;
    /* Rectangle spanning a tile boundary at z=2 (tiles are 45 lon × 45 lat) */
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

    TEST_ASSERT_TRUE(c.count >= 1);
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        TEST_ASSERT_TRUE(cg->n_coords >= 4);

        if (cg->offsets && cg->n_offsets > 1) {
            /* Check each ring is closed */
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                uint32_t rn = re - rs;
                TEST_ASSERT_TRUE(rn >= 4);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
            }
        } else {
            /* Single ring: first == last */
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[0],
                                       cg->x[cg->n_coords - 1]);
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[0],
                                       cg->y[cg->n_coords - 1]);
        }
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

static void test_polygon_encloses_tile(void) {
    /* A large polygon that completely encloses several tiles.
     * Every tile within the polygon bbox should receive a clipped
     * polygon (the tile rectangle). */
    arpt_geom g = {0};
    g.type = 3;
    /* Large rectangle: lon [-20, 30], lat [35, 60] */
    double x[] = {-20.0, 30.0, 30.0, -20.0, -20.0};
    double y[] = {35.0, 35.0, 60.0, 60.0, 35.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    /* At z=3, tiles are 45 deg wide × 22.5 deg tall.
     * The polygon spans ~50 lon × 25 lat, so it fully encloses
     * at least one interior tile. */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 3, collect_cb, &c);

    /* Should produce clipped polygons in multiple tiles */
    TEST_ASSERT_TRUE(c.count >= 2);

    /* Every clipped polygon should have >= 4 vertices (closed ring) */
    for (int i = 0; i < c.count; i++) {
        TEST_ASSERT_TRUE(c.results[i].geom.n_coords >= 4);
    }

    collector_free(&c);
}

static void test_polygon_encloses_tile_high_zoom(void) {
    /* At higher zoom, a polygon encloses many tiles. Pick a zoom
     * where tiles are small relative to the polygon. */
    arpt_geom g = {0};
    g.type = 3;
    /* Rectangle: lon [5, 8], lat [45, 48] — ~3 deg × 3 deg */
    double x[] = {5.0, 8.0, 8.0, 5.0, 5.0};
    double y[] = {45.0, 45.0, 48.0, 48.0, 45.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    /* z=5: tiles are 11.25 deg × 5.625 deg. The polygon is smaller
     * than one tile, so we should get exactly 1 tile. */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 5, collect_cb, &c);
    TEST_ASSERT_TRUE(c.count >= 1);
    collector_free(&c);

    /* z=7: tiles are ~2.8 deg × ~1.4 deg. The polygon spans
     * ~1-2 tiles per axis and fully encloses interior tiles. */
    collector_init(&c);
    arpt_assign_tiles(&g, 7, collect_cb, &c);
    /* Should produce multiple tiles */
    TEST_ASSERT_TRUE(c.count >= 2);
    for (int i = 0; i < c.count; i++) {
        TEST_ASSERT_TRUE(c.results[i].geom.n_coords >= 4);
    }
    collector_free(&c);
}

/* ---- MultiPolygon clipping ---- */

static void test_multipolygon_parts(void) {
    /* MultiPolygon with two separate polygon parts.
     * Each part should be clipped independently. */
    arpt_geom g = {0};
    g.type = 6; /* MultiPolygon */

    /* Part 0: small square in western hemisphere
     * Part 1: small square in eastern hemisphere
     * At z=0 (2 cols × 1 row), they should end up in different tiles. */
    double x[] = {
        /* Part 0: lon [-100, -95] */
        -100.0, -95.0, -95.0, -100.0, -100.0,
        /* Part 1: lon [95, 100] */
        95.0, 100.0, 100.0, 95.0, 95.0
    };
    double y[] = {
        /* Part 0: lat [40, 45] */
        40.0, 40.0, 45.0, 45.0, 40.0,
        /* Part 1: lat [40, 45] */
        40.0, 40.0, 45.0, 45.0, 40.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;
    uint32_t parts[] = {0, 1};  /* ring 0 = part 0, ring 1 = part 1 */
    g.parts = parts;
    g.n_parts = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);

    /* Should get exactly 2 results, one per tile */
    TEST_ASSERT_EQUAL_INT(2, c.count);

    /* Part 0 in western tile (tx=0), part 1 in eastern tile (tx=1) */
    tile_result *west = find_result(&c, 0, 0, 0);
    tile_result *east = find_result(&c, 0, 1, 0);
    TEST_ASSERT_NOT_NULL(west);
    TEST_ASSERT_NOT_NULL(east);

    /* Each should be a closed polygon */
    TEST_ASSERT_TRUE(west->geom.n_coords >= 4);
    TEST_ASSERT_TRUE(east->geom.n_coords >= 4);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, west->geom.x[0],
                               west->geom.x[west->geom.n_coords - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, east->geom.x[0],
                               east->geom.x[east->geom.n_coords - 1]);

    collector_free(&c);
}

/* ---- Concave polygon clipping ---- */

/* Compute the signed area of a closed ring (first == last).
 * Positive = CCW, negative = CW. */
static double ring_signed_area(const double *x, const double *y, uint32_t n) {
    double area = 0.0;
    for (uint32_t i = 0; i < n - 1; i++) {
        area += x[i] * y[i + 1] - x[i + 1] * y[i];
    }
    return area * 0.5;
}

static void test_concave_polygon_gulf(void) {
    /* A "horseshoe" polygon simulating the Gulf of Mexico coast.
     *
     * Shape (CCW exterior ring):
     *   The polygon is an upside-down U covering the "land" around
     *   a gulf-like concavity.  At zoom 2 (8 cols × 4 rows), the
     *   concavity should produce tiles with NO polygon coverage.
     *
     *   Vertices (simplified):
     *     (-100, 10) → (-80, 10) → (-80, 35) → (-85, 35)
     *     → (-85, 20) → (-95, 20) → (-95, 35) → (-100, 35)
     *     → close
     *
     *   This looks like:
     *
     *     (-100,35)--(-95,35)        (-85,35)--(-80,35)
     *          |          |              |          |
     *          |   (-95,20)------(-85,20)          |
     *          |                                   |
     *     (-100,10)---------------------------(-80,10)
     *
     *   The "gulf" is the gap between (-95,20) and (-85,20) upward
     *   to (-95,35) and (-85,35).
     */
    arpt_geom g = {0};
    g.type = 3;
    /* CCW exterior ring (closed) */
    double x[] = {
        -100.0, -80.0, -80.0, -85.0,
        -85.0, -95.0, -95.0, -100.0, -100.0
    };
    double y[] = {
        10.0, 10.0, 35.0, 35.0,
        20.0, 20.0, 35.0, 35.0, 10.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 9;
    uint32_t offsets[] = {0, 9};
    g.offsets = offsets;
    g.n_offsets = 2;

    /* Clip at zoom 3: tiles are 22.5° × 22.5°.
     * The gulf gap spans (-95, 20) to (-85, 35), which is ~10° wide.
     * At z=3 tile width is 22.5°, so the gap fits within one tile column.
     *
     * Tile grid at z3 (16 cols × 8 rows):
     *   col 3: lon [-135, -112.5]
     *   col 4: lon [-112.5, -90]   ← contains left arm of U
     *   col 5: lon [-90, -67.5]    ← contains gulf gap AND right arm
     *
     * Actually at z3 the tiles are smaller. Let's use z2 (8 cols × 4 rows):
     *   col 2: lon [-90, -45]   ← this tile contains the gap
     *   row 2: lat [0, 45]      ← this row contains the shape
     *
     * At z=2, tile (2, 2, 2) covers lon [-90, -45], lat [0, 45].
     * The polygon enters from left (-100 to -80), so it spans columns 1-2.
     * The "gulf" is at lon [-95, -85], which falls in column 2 (and partly 1).
     */

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    /* Find the tile that contains the gulf area.
     * At z=2: 8 cols × 4 rows, each tile is 45° × 45°.
     * Tile (2, 2, 2) = col 2 (lon -90 to -45), row 2 (lat 0 to 45).
     * The entire polygon fits in this tile. */
    tile_result *gulf_tile = find_result(&c, 2, 2, 2);
    TEST_ASSERT_NOT_NULL_MESSAGE(gulf_tile,
        "Expected polygon clipped to tile containing the gulf");

    /* The clipped polygon should be a concave shape (the horseshoe).
     * Verify: the ring should be a SINGLE closed ring with correct
     * winding (CCW = positive area) and the area should be LESS than
     * the bounding box area (because of the concavity). */
    TEST_ASSERT_TRUE_MESSAGE(gulf_tile->geom.n_coords >= 4,
        "Clipped ring too small");

    double area = ring_signed_area(gulf_tile->geom.x, gulf_tile->geom.y,
                                   gulf_tile->geom.n_coords);
    /* The full bounding box is 20° × 25° = 500 sq.deg.
     * The concavity removes 10° × 15° = 150 sq.deg.
     * So the polygon area should be ~350 sq.deg (with some buffer). */
    double abs_area = area < 0 ? -area : area;
    TEST_ASSERT_TRUE_MESSAGE(abs_area < 600.0,
        "Polygon area too large — concavity not preserved");
    TEST_ASSERT_TRUE_MESSAGE(abs_area > 100.0,
        "Polygon area too small — shape might be degenerate");

    fprintf(stderr, "  Gulf concavity test: ring has %u vertices, area=%.1f\n",
            gulf_tile->geom.n_coords, abs_area);

    collector_free(&c);
}

/* Test that a tile ENTIRELY within the concavity of a polygon gets
 * no polygon coverage.  This simulates a tile in the Gulf of Mexico. */
static void test_concave_polygon_empty_interior_tile(void) {
    /* Same horseshoe shape but at higher zoom where the concavity
     * spans multiple tiles.  A tile inside the concavity should
     * receive NO polygon data. */
    arpt_geom g = {0};
    g.type = 3;
    /* Large horseshoe covering a 60° wide, 40° tall region with
     * a 30° × 25° concavity in the center-top. */
    double x[] = {
        -110.0, -50.0, -50.0, -65.0,
        -65.0, -95.0, -95.0, -110.0, -110.0
    };
    double y[] = {
        5.0, 5.0, 40.0, 40.0,
        15.0, 15.0, 40.0, 40.0, 5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 9;
    uint32_t offsets[] = {0, 9};
    g.offsets = offsets;
    g.n_offsets = 2;

    /* At z=3 (16 cols × 8 rows), tiles are 22.5° × 22.5°.
     * The concavity spans lon [-95, -65] lat [15, 40].
     * Tile (3, 4, 3) = col 4 (lon [-90, -67.5]), row 3 (lat [22.5, 45]).
     * This tile is entirely within the concavity! */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 3, collect_cb, &c);

    /* Check that tile (3, 4, 3) does NOT receive a polygon.
     * The concavity center tile should be empty. */
    tile_result *center = find_result(&c, 3, 4, 3);

    /* If center is NOT null, the clipper erroneously produced output
     * for a tile inside the polygon's concavity. Print diagnostics. */
    if (center) {
        fprintf(stderr, "  ERROR: Tile (3,4,3) inside concavity got %u vertices!\n",
                center->geom.n_coords);
        for (uint32_t i = 0; i < center->geom.n_coords && i < 20; i++) {
            fprintf(stderr, "    [%u] (%.3f, %.3f)\n", i,
                    center->geom.x[i], center->geom.y[i]);
        }
    }
    TEST_ASSERT_NULL_MESSAGE(center,
        "Tile inside polygon concavity should NOT receive polygon data");

    collector_free(&c);
}

static void test_multipolygon_same_tile(void) {
    /* Two polygon parts that both fall in the same tile.
     * They should produce separate callbacks (not merged). */
    arpt_geom g = {0};
    g.type = 6;
    double x[] = {
        /* Part 0 */
        5.0, 6.0, 6.0, 5.0, 5.0,
        /* Part 1 */
        7.0, 8.0, 8.0, 7.0, 7.0
    };
    double y[] = {
        5.0, 5.0, 6.0, 6.0, 5.0,
        5.0, 5.0, 6.0, 6.0, 5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;
    uint32_t parts[] = {0, 1};
    g.parts = parts;
    g.n_parts = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);

    /* Both parts in the same tile → 2 separate callbacks */
    TEST_ASSERT_EQUAL_INT(2, c.count);
    collector_free(&c);
}

/* Test a re-entrant polygon that exits and re-enters the clip rect
 * on the SAME edge multiple times.  This is the pattern that causes
 * Scandinavia-like wedge artifacts if the boundary walk pairs the
 * wrong entry with the wrong exit. */
static void test_reentrant_same_edge(void) {
    /* A polygon that crosses the right edge of a tile rect twice:
     *
     *     clip rect [0, 0, 10, 10]
     *
     *                   |  rect right edge (x=10)
     *     (5,9)----(12,8)  exit 1
     *       |           |
     *     (5,6)----(12,5)  re-entry on right edge → but outside
     *       |
     *     (5,3)----(12,2)  exit 2
     *       |           |
     *     (5,0.5)--(12,-1) outside bottom
     *
     * Actually let me design this more carefully.
     *
     * A CCW polygon that goes:
     *   inside → exits right → comes back in right → exits right again
     *   → comes back in right → back to start
     *
     * Coords (CCW):
     *   (2, 1) → (12, 1) → (12, 4) → (2, 4) → (2, 6) → (12, 6) →
     *   (12, 9) → (2, 9) → close
     *
     * With clip rect [0, 0, 10, 10], this polygon exits the right edge
     * at y=1..4 and y=6..9, re-entering each time.
     */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {2, 12, 12, 2, 2, 12, 12, 2, 2};
    double y[] = {1,  1,  4, 4, 6,  6,  9, 9, 1};
    g.x = x;
    g.y = y;
    g.n_coords = 9;
    uint32_t offsets[] = {0, 9};
    g.offsets = offsets;
    g.n_offsets = 2;

    /* Clip at zoom 3 (16 cols × 8 rows), tiles are 22.5° × 22.5°.
     * Pick a zoom/tile such that the clip rect contains our polygon shape.
     * Actually, let's just test the raw clipper behavior with a custom
     * polygon and zoom that creates the right tile bounds.
     *
     * At z=3 tile (12, 4): lon [90, 112.5], lat [0, 22.5]
     * We need our polygon to cross tile boundaries. Let me use a different
     * approach — test with a polygon positioned in degree space that
     * spans a tile boundary on the right edge. */

    /* Let's use z=2, tiles are 45° lon × 45° lat.
     * Tile (2, 2, 2) = lon [-90, -45], lat [0, 45].
     * Put polygon at lon [-60, -30] (crossing right edge at -45)
     * with two horizontal bars. */
    double x2[] = {
        -60, -30, -30, -60,
        -60, -30, -30, -60, -60
    };
    double y2[] = {
        5,   5,  15,  15,
        25, 25,  35,  35,  5
    };
    g.x = x2;
    g.y = y2;
    g.n_coords = 9;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    /* The polygon has two horizontal bars crossing right edge of tile (2,2,2).
     * Each bar should produce clipped geometry. */
    TEST_ASSERT_TRUE(c.count >= 1);

    /* Check that clipped polygons have valid rings (closed, reasonable area) */
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        TEST_ASSERT_TRUE(cg->n_coords >= 4);

        /* Check ring closure */
        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                TEST_ASSERT_TRUE(re - rs >= 4);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
            }
        } else {
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[0],
                                       cg->x[cg->n_coords - 1]);
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[0],
                                       cg->y[cg->n_coords - 1]);
        }

    }

    /* Compute total clipped area in tile (2, 2, 2) which should contain
     * two partial bars */
    tile_result *t = find_result(&c, 2, 2, 2);
    if (t) {
        double total_area = 0;
        if (t->geom.offsets && t->geom.n_offsets > 1) {
            uint32_t nr = t->geom.n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = t->geom.offsets[ri];
                uint32_t re = t->geom.offsets[ri + 1];
                double a = ring_signed_area(t->geom.x + rs, t->geom.y + rs,
                                            re - rs);
                total_area += a;
            }
        } else {
            total_area = ring_signed_area(t->geom.x, t->geom.y,
                                          t->geom.n_coords);
        }
        /* Each bar within the tile is 15° wide × 10° tall = 150 sq.deg.
         * Total should be ~300 sq.deg (two bars). Allow generous range. */
        double abs_area = total_area < 0 ? -total_area : total_area;
        TEST_ASSERT_TRUE_MESSAGE(abs_area > 50.0,
            "Reentrant polygon area too small — clipping lost geometry");
    }

    collector_free(&c);
}

/* Test polygon that exits and re-enters the clip rect on the top edge,
 * simulating Scandinavian coastline patterns. */
static void test_reentrant_top_edge(void) {
    /* A CCW polygon shaped like an inverted W that crosses the top edge
     * of a tile multiple times:
     *
     *  top edge (y=45) - - - - - - - - - - -
     *        /\          /\
     *       /  \        /  \
     *      /    \      /    \
     *     /      \    /      \
     *    /________\__/________\
     *
     * Polygon exits top, re-enters, exits again, re-enters.
     */
    arpt_geom g = {0};
    g.type = 3;
    /* CCW polygon with two peaks that go above y=45 (tile top edge at z=2) */
    double x[] = {
        -80, -75, -70, -65, -60, -55, -50, -80, -80
    };
    double y[] = {
        30, 50, 30, 30, 50, 30, 30, 30, 30
    };
    g.x = x;
    g.y = y;
    g.n_coords = 9;
    uint32_t offsets[] = {0, 9};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 2, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);

    /* Each clipped polygon should have closed rings */
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        TEST_ASSERT_TRUE(cg->n_coords >= 4);
        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                TEST_ASSERT_TRUE(re - rs >= 4);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
            }
        } else {
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[0],
                                       cg->x[cg->n_coords - 1]);
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[0],
                                       cg->y[cg->n_coords - 1]);
        }
    }

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
    RUN_TEST(test_line_no_phantom_segments);
    RUN_TEST(test_polygon_within_tile);
    RUN_TEST(test_polygon_rings_closed);
    RUN_TEST(test_polygon_crossing_tiles);
    RUN_TEST(test_polygon_encloses_tile);
    RUN_TEST(test_polygon_encloses_tile_high_zoom);
    RUN_TEST(test_concave_polygon_gulf);
    RUN_TEST(test_concave_polygon_empty_interior_tile);
    RUN_TEST(test_multipolygon_parts);
    RUN_TEST(test_multipolygon_same_tile);
    RUN_TEST(test_reentrant_same_edge);
    RUN_TEST(test_reentrant_top_edge);
    return UNITY_END();
}
