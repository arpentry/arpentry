#include "unity.h"
#include "clip.h"
#include "simplify.h"
#include <stdbool.h>
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
    arpt_assign_tiles(NULL, NULL, 0, NULL, NULL);
}

static void test_assign_tiles_null_cb(void) {
    arpt_geom g = {0};
    g.type = 1;
    double x = 6.6, y = 46.5;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;
    arpt_assign_tiles(&g, &g, 0, NULL, NULL);
}

static void test_assign_tiles_empty_geom(void) {
    arpt_geom g = {0};
    g.type = 1;
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 0, collect_cb, &c);
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
    arpt_assign_tiles(&g, &g, 0, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 1, collect_cb, &c);

    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].z);
    TEST_ASSERT_EQUAL_INT(2, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].y);
    collector_free(&c);
}

static void test_multipoint(void) {
    /* Two points at different locations */
    arpt_geom g = {0};
    g.type = 1; /* Point (flattened MultiPoint) */
    double x[] = {-90.0, 90.0};
    double y[] = {0.0, 0.0};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 1, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 4, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 5, collect_cb, &c);
    TEST_ASSERT_TRUE(c.count >= 1);
    collector_free(&c);

    /* z=7: tiles are ~2.8 deg × ~1.4 deg. The polygon spans
     * ~1-2 tiles per axis and fully encloses interior tiles. */
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 7, collect_cb, &c);
    /* Should produce multiple tiles */
    TEST_ASSERT_TRUE(c.count >= 2);
    for (int i = 0; i < c.count; i++) {
        TEST_ASSERT_TRUE(c.results[i].geom.n_coords >= 4);
    }
    collector_free(&c);
}

/* ---- MultiPolygon clipping ---- */

static void test_multipolygon_parts(void) {
    /* Flattened MultiPolygon: two rings from separate polygon parts.
     * Both rings are exterior (CCW) — each clipped independently as
     * rings of one Polygon (even-odd fill handles correctness). */
    arpt_geom g = {0};
    g.type = 3; /* Polygon (flattened from MultiPolygon) */

    /* Ring 0: small square in western hemisphere
     * Ring 1: small square in eastern hemisphere
     * At z=0 (2 cols × 1 row), they should end up in different tiles. */
    double x[] = {
        /* Ring 0: lon [-100, -95] */
        -100.0, -95.0, -95.0, -100.0, -100.0,
        /* Ring 1: lon [95, 100] */
        95.0, 100.0, 100.0, 95.0, 95.0
    };
    double y[] = {
        /* Ring 0: lat [40, 45] */
        40.0, 40.0, 45.0, 45.0, 40.0,
        /* Ring 1: lat [40, 45] */
        40.0, 40.0, 45.0, 45.0, 40.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 0, collect_cb, &c);

    /* Should get exactly 2 results, one per tile */
    TEST_ASSERT_EQUAL_INT(2, c.count);

    /* Ring 0 in western tile (tx=0), ring 1 in eastern tile (tx=1) */
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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

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
    g.type = 3;
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
    /* parts removed: multi-types flattened at parse time */

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 0, collect_cb, &c);

    /* Flattened polygon with 2 rings in the same tile → 1 callback */
    TEST_ASSERT_EQUAL_INT(1, c.count);
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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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

/* ---- Polygon with hole clipping ---- */

/* Test a polygon with a hole entirely within one tile.
 * The hole ring should be preserved in the clipped output. */
static void test_polygon_with_hole_within_tile(void) {
    arpt_geom g = {0};
    g.type = 3; /* Polygon */
    /* Exterior ring (CCW): 20° × 20° square centered at (10, 50) */
    /* Hole ring (CW): 10° × 10° square inside */
    double x[] = {
        /* Exterior CCW */
        0.0, 20.0, 20.0, 0.0, 0.0,
        /* Hole CW */
        5.0, 5.0, 15.0, 15.0, 5.0
    };
    double y[] = {
        40.0, 40.0, 60.0, 60.0, 40.0,
        45.0, 55.0, 55.0, 45.0, 45.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    /* At z=2 (8 cols × 4 rows), tiles are 45° × 45°.
     * The whole polygon fits in one tile. */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);

    /* Find a tile that got clipped geometry */
    bool found_two_rings = false;
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        if (cg->offsets && cg->n_offsets >= 3) {
            found_two_rings = true;
            uint32_t nr = cg->n_offsets - 1;
            TEST_ASSERT_EQUAL_UINT32(2, nr);

            /* Both rings should be closed */
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                uint32_t rn = re - rs;
                TEST_ASSERT_TRUE_MESSAGE(rn >= 4,
                    "Ring too small after clipping");
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
            }

            /* Exterior ring should have larger absolute area than hole */
            uint32_t ext_s = cg->offsets[0], ext_e = cg->offsets[1];
            uint32_t hole_s = cg->offsets[1], hole_e = cg->offsets[2];
            double ext_area = ring_signed_area(cg->x + ext_s, cg->y + ext_s,
                                                ext_e - ext_s);
            double hole_area = ring_signed_area(cg->x + hole_s, cg->y + hole_s,
                                                 hole_e - hole_s);
            double ext_abs = ext_area < 0 ? -ext_area : ext_area;
            double hole_abs = hole_area < 0 ? -hole_area : hole_area;
            TEST_ASSERT_TRUE_MESSAGE(ext_abs > hole_abs,
                "Exterior ring should be larger than hole");

            /* Exterior and hole should have opposite winding */
            TEST_ASSERT_TRUE_MESSAGE(
                (ext_area > 0 && hole_area < 0) ||
                (ext_area < 0 && hole_area > 0),
                "Exterior and hole should have opposite winding");
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_two_rings,
        "Polygon with hole should produce a clipped geometry with 2 rings");

    collector_free(&c);
}

/* Test a polygon with a hole that crosses a tile boundary.
 * Both the exterior and hole should be properly clipped. */
static void test_polygon_with_hole_crossing_tile(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Exterior ring spans across the lon=0 tile boundary at z=2.
     * At z=2: tile boundary at lon -45, 0, 45.
     * Exterior: lon [-10, 10], lat [20, 40] — crosses lon=0.
     * Hole: lon [-5, 5], lat [25, 35] — also crosses lon=0. */
    double x[] = {
        /* Exterior CCW */
        -10.0, 10.0, 10.0, -10.0, -10.0,
        /* Hole CW */
        -5.0, -5.0, 5.0, 5.0, -5.0
    };
    double y[] = {
        20.0, 20.0, 40.0, 40.0, 20.0,
        25.0, 35.0, 35.0, 25.0, 25.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

    /* Should produce results in at least 2 tiles (left and right of lon=0).
     * Actually at z=2 the tile boundary is at lon=-45 and lon=0,
     * so with buffer the polygon at [-10,10] might all fit in one tile.
     * Use z=3 instead. */
    collector_free(&c);

    /* At z=3 (16 cols × 8 rows), tiles are 22.5° × 22.5°.
     * Tile boundaries at lon ..., -22.5, 0, 22.5, ...
     * The polygon [-10, 10] crosses lon=0. */
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);

    /* Every clipped polygon should have closed rings */
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        TEST_ASSERT_TRUE(cg->n_coords >= 4);

        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                uint32_t rn = re - rs;
                TEST_ASSERT_TRUE_MESSAGE(rn >= 4,
                    "Clipped ring too small");
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
            }
        }
    }

    collector_free(&c);
}

/* Test that a tile entirely within the hole of a polygon gets no coverage. */
static void test_polygon_hole_empty_interior(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Large exterior with large hole. At z=3 (22.5° tiles), the hole
     * should span at least one full tile.
     * Exterior: lon [-30, 30], lat [10, 60] (60° × 50°)
     * Hole: lon [-15, 15], lat [20, 50] (30° × 30°)
     * At z=3 tiles are 22.5° — the hole spans >1 tile. */
    double x[] = {
        /* Exterior CCW */
        -30.0, 30.0, 30.0, -30.0, -30.0,
        /* Hole CW */
        -15.0, -15.0, 15.0, 15.0, -15.0
    };
    double y[] = {
        10.0, 10.0, 60.0, 60.0, 10.0,
        20.0, 50.0, 50.0, 20.0, 20.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);

    /* For tiles that got both rings, the net area (exterior - hole)
     * should be less than the exterior alone. */
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        if (cg->offsets && cg->n_offsets >= 3) {
            uint32_t ext_s = cg->offsets[0], ext_e = cg->offsets[1];
            uint32_t hole_s = cg->offsets[1], hole_e = cg->offsets[2];
            double ext_area = ring_signed_area(cg->x + ext_s, cg->y + ext_s,
                                                ext_e - ext_s);
            double hole_area = ring_signed_area(cg->x + hole_s, cg->y + hole_s,
                                                 hole_e - hole_s);
            /* Net area should be positive and less than exterior */
            double net = ext_area + hole_area; /* hole_area is negative if CW */
            double ext_abs = ext_area < 0 ? -ext_area : ext_area;
            double net_abs = net < 0 ? -net : net;
            TEST_ASSERT_TRUE_MESSAGE(net_abs < ext_abs,
                "Net area with hole should be less than exterior alone");
            TEST_ASSERT_TRUE_MESSAGE(net_abs > 0.1,
                "Net area should be positive (hole shouldn't consume exterior)");
        }
    }

    collector_free(&c);
}

/* Test a tile that only sees the hole (exterior clips to tile, hole also clips,
 * but tile is mostly inside the hole — net area is small or exterior-only). */
static void test_polygon_hole_only_frame(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Exterior covers a large area, hole is nearly as big.
     * The "frame" between exterior and hole is thin.
     * At high zoom, some tiles only see the exterior frame. */
    /* Exterior: lon [-5, 25], lat [40, 55] (30° × 15°)
     * Hole: lon [-2, 22], lat [42, 53] (24° × 11°)
     * Frame is 3° on left/right, 2° on top/bottom. */
    double x[] = {
        /* Exterior CCW */
        -5.0, 25.0, 25.0, -5.0, -5.0,
        /* Hole CW */
        -2.0, -2.0, 22.0, 22.0, -2.0
    };
    double y[] = {
        40.0, 40.0, 55.0, 55.0, 40.0,
        42.0, 53.0, 53.0, 42.0, 42.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 4, collect_cb, &c);

    /* At z=4, tiles are 22.5° × 11.25°. Some tiles will see the frame.
     * Verify all clipped rings are valid. */
    TEST_ASSERT_TRUE(c.count >= 1);
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
        }
    }

    collector_free(&c);
}

/* ---- MultiPolygon with hole and island ---- */

/* Classic GIS pattern: a lake with an island.
 * Part 0: land mass with lake (exterior + hole)
 * Part 1: island inside the lake (small polygon inside the hole) */
static void test_multipolygon_hole_and_island_within_tile(void) {
    arpt_geom g = {0};
    g.type = 3; /* Polygon (flattened MultiPolygon) */

    /* Part 0: Land with lake
     *   Exterior CCW: lon [0, 20], lat [0, 20]
     *   Hole CW (lake): lon [5, 15], lat [5, 15]
     * Part 1: Island
     *   Exterior CCW: lon [8, 12], lat [8, 12] (inside the hole) */
    double x[] = {
        /* Part 0, Ring 0: Exterior CCW */
        0.0, 20.0, 20.0, 0.0, 0.0,
        /* Part 0, Ring 1: Hole CW (lake) */
        5.0, 5.0, 15.0, 15.0, 5.0,
        /* Part 1, Ring 0: Island CCW */
        8.0, 12.0, 12.0, 8.0, 8.0
    };
    double y[] = {
        0.0, 0.0, 20.0, 20.0, 0.0,
        5.0, 15.0, 15.0, 5.0, 5.0,
        8.0, 8.0, 12.0, 12.0, 8.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 5, 10, 15};
    g.offsets = offsets;
    g.n_offsets = 4;
    /* Part 0 starts at ring 0, Part 1 starts at ring 2 */
    /* parts removed: multi-types flattened at parse time */

    /* At z=2 (45° tiles), everything fits in one tile.
     * Flattened polygon: all 3 rings in one callback. */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

    TEST_ASSERT_EQUAL_INT_MESSAGE(1, c.count,
        "Flattened polygon should produce 1 callback with all rings");

    arpt_geom *cg = &c.results[0].geom;
    TEST_ASSERT_NOT_NULL(cg->offsets);
    uint32_t nr = cg->n_offsets - 1;
    TEST_ASSERT_EQUAL_UINT32(3, nr);  /* exterior + hole + island */

    /* All rings should be closed */
    for (uint32_t ri = 0; ri < nr; ri++) {
        uint32_t rs = cg->offsets[ri];
        uint32_t re = cg->offsets[ri + 1];
        TEST_ASSERT_TRUE(re - rs >= 4);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
    }

    collector_free(&c);
}

/* MultiPolygon with hole and island crossing a tile boundary.
 * The exterior, hole, and island all straddle the tile edge. */
static void test_multipolygon_hole_and_island_crossing_tile(void) {
    arpt_geom g = {0};
    g.type = 3;

    /* At z=3, tile boundaries at lon ..., -22.5, 0, 22.5, ...
     * Place geometry crossing lon=0.
     *
     * Part 0: Land with lake
     *   Exterior: lon [-15, 15], lat [20, 45] — crosses lon=0
     *   Hole (lake): lon [-8, 8], lat [25, 40] — also crosses lon=0
     * Part 1: Island inside lake
     *   lon [-3, 3], lat [30, 35] — crosses lon=0 */
    double x[] = {
        /* Part 0, Exterior CCW */
        -15.0, 15.0, 15.0, -15.0, -15.0,
        /* Part 0, Hole CW */
        -8.0, -8.0, 8.0, 8.0, -8.0,
        /* Part 1, Island CCW */
        -3.0, 3.0, 3.0, -3.0, -3.0
    };
    double y[] = {
        20.0, 20.0, 45.0, 45.0, 20.0,
        25.0, 40.0, 40.0, 25.0, 25.0,
        30.0, 30.0, 35.0, 35.0, 30.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 5, 10, 15};
    g.offsets = offsets;
    g.n_offsets = 4;
    /* parts removed: multi-types flattened at parse time */

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Should produce results in multiple tiles */
    TEST_ASSERT_TRUE_MESSAGE(c.count >= 2,
        "Geometry crossing tile boundary should produce multiple tiles");

    /* Verify all clipped rings are valid */
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        TEST_ASSERT_TRUE(cg->n_coords >= 4);

        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                uint32_t rn = re - rs;
                TEST_ASSERT_TRUE_MESSAGE(rn >= 4,
                    "Clipped ring degenerate after tile boundary clip");
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

/* MultiPolygon with hole + island at high zoom where each component
 * ends up in different tiles. */
static void test_multipolygon_hole_island_high_zoom(void) {
    arpt_geom g = {0};
    g.type = 3;

    /* At z=7, tiles are ~2.8° × ~1.4°.
     * Part 0: Large land mass lon [-5, 5], lat [44, 50] with
     *   hole lon [-2, 2], lat [46, 48]
     * Part 1: Small island lon [-0.5, 0.5], lat [46.8, 47.2] */
    double x[] = {
        /* Part 0, Exterior CCW */
        -5.0, 5.0, 5.0, -5.0, -5.0,
        /* Part 0, Hole CW */
        -2.0, -2.0, 2.0, 2.0, -2.0,
        /* Part 1, Island CCW */
        -0.5, 0.5, 0.5, -0.5, -0.5
    };
    double y[] = {
        44.0, 44.0, 50.0, 50.0, 44.0,
        46.0, 48.0, 48.0, 46.0, 46.0,
        46.8, 46.8, 47.2, 47.2, 46.8
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 5, 10, 15};
    g.offsets = offsets;
    g.n_offsets = 4;
    /* parts removed: multi-types flattened at parse time */

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 7, collect_cb, &c);

    /* At this zoom there should be many tiles */
    TEST_ASSERT_TRUE_MESSAGE(c.count >= 4,
        "High zoom MultiPolygon should produce many tile results");

    /* Validate all results */
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        TEST_ASSERT_TRUE(cg->n_coords >= 4);

        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                uint32_t rn = re - rs;
                TEST_ASSERT_TRUE_MESSAGE(rn >= 4, "Degenerate ring at high zoom");
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->x[rs], cg->x[re - 1]);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, cg->y[rs], cg->y[re - 1]);
            }

            /* If we have 2 rings, check winding consistency */
            if (nr == 2) {
                uint32_t ext_s = cg->offsets[0], ext_e = cg->offsets[1];
                uint32_t hole_s = cg->offsets[1], hole_e = cg->offsets[2];
                double ext_a = ring_signed_area(cg->x + ext_s, cg->y + ext_s,
                                                 ext_e - ext_s);
                double hole_a = ring_signed_area(cg->x + hole_s, cg->y + hole_s,
                                                  hole_e - hole_s);
                TEST_ASSERT_TRUE_MESSAGE(
                    (ext_a > 0 && hole_a < 0) || (ext_a < 0 && hole_a > 0),
                    "Exterior and hole must have opposite winding at high zoom");
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

/* Test that the area accounting is correct: for every tile, the total
 * polygon area (exterior - holes + islands) should be consistent. */
static void test_multipolygon_hole_island_area_accounting(void) {
    arpt_geom g = {0};
    g.type = 3;

    /* Part 0: 20° × 20° land with 10° × 10° lake
     * Part 1: 4° × 4° island
     * Expected: land 400 - lake 100 + island 16 = 316 sq.deg total */
    double x[] = {
        /* Part 0, Exterior CCW */
        0.0, 20.0, 20.0, 0.0, 0.0,
        /* Part 0, Hole CW */
        5.0, 5.0, 15.0, 15.0, 5.0,
        /* Part 1, Island CCW */
        8.0, 12.0, 12.0, 8.0, 8.0
    };
    double y[] = {
        0.0, 0.0, 20.0, 20.0, 0.0,
        5.0, 15.0, 15.0, 5.0, 5.0,
        8.0, 8.0, 12.0, 12.0, 8.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 5, 10, 15};
    g.offsets = offsets;
    g.n_offsets = 4;
    /* parts removed: multi-types flattened at parse time */

    /* At z=2 (45° tiles), everything fits in one tile.
     * Sum all ring areas across all callbacks. */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

    double total_area = 0.0;
    for (int i = 0; i < c.count; i++) {
        arpt_geom *cg = &c.results[i].geom;
        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                total_area += ring_signed_area(cg->x + rs, cg->y + rs,
                                                re - rs);
            }
        } else {
            total_area += ring_signed_area(cg->x, cg->y, cg->n_coords);
        }
    }

    /* Expected: 400 (ext) - 100 (hole) + 16 (island) = 316
     * The sign depends on winding. Take absolute value of net. */
    double abs_total = total_area < 0 ? -total_area : total_area;
    fprintf(stderr, "  Area accounting: total=%.1f (expected ~316)\n", abs_total);
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(20.0, 316.0, abs_total,
        "Total area should be exterior - hole + island");

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
    arpt_assign_tiles(&g, &g, 2, collect_cb, &c);

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

/* ---- Geometry completely encloses tile ---- */

/* A polygon that is strictly larger than the tile on all four sides.
 * The clipped output should be the tile's buffered rectangle. */
static void test_polygon_encloses_tile_completely(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* At z=3: 16 cols × 8 rows, tiles are 22.5° × 22.5°.
     * Tile (8, 4) = lon [0, 22.5], lat [0, 22.5].
     * Buffer = 22.5 * 8/256 ≈ 0.703° per side.
     * Buffered bounds ≈ [-0.703, -0.703, 23.203, 23.203].
     *
     * Polygon: lon [-10, 35], lat [-10, 35] — 45° × 45°,
     * much larger than the tile on every side. */
    double x[] = {-10.0, 35.0, 35.0, -10.0, -10.0};
    double y[] = {-10.0, -10.0, 35.0, 35.0, -10.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Tile (3, 8, 4) should definitely be present */
    tile_result *t = find_result(&c, 3, 8, 4);
    TEST_ASSERT_NOT_NULL_MESSAGE(t,
        "Tile fully enclosed by polygon should receive geometry");

    /* The clipped polygon should be a closed ring */
    TEST_ASSERT_TRUE(t->geom.n_coords >= 4);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, t->geom.x[0],
                               t->geom.x[t->geom.n_coords - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, t->geom.y[0],
                               t->geom.y[t->geom.n_coords - 1]);

    /* The clipped area should approximate the buffered tile area.
     * Tile is 22.5° × 22.5° = 506.25 sq.deg.
     * Buffered tile is ~23.906° × 23.906° ≈ 571.5 sq.deg.
     * Allow generous tolerance since the polygon vertices get clipped
     * to the buffered bounds. */
    double area = ring_signed_area(t->geom.x, t->geom.y, t->geom.n_coords);
    double abs_area = area < 0 ? -area : area;
    TEST_ASSERT_TRUE_MESSAGE(abs_area > 400.0,
        "Enclosed tile clipped area too small");
    TEST_ASSERT_TRUE_MESSAGE(abs_area < 700.0,
        "Enclosed tile clipped area too large");

    /* All vertices should be within or very near the buffered tile bounds */
    double buf = 22.5 * 8.0 / 256.0;
    double bmin_x = 0.0 - buf, bmax_x = 22.5 + buf;
    double bmin_y = 0.0 - buf, bmax_y = 22.5 + buf;
    for (uint32_t i = 0; i < t->geom.n_coords; i++) {
        TEST_ASSERT_TRUE_MESSAGE(
            t->geom.x[i] >= bmin_x - 0.01 && t->geom.x[i] <= bmax_x + 0.01,
            "Clipped vertex x outside buffered tile bounds");
        TEST_ASSERT_TRUE_MESSAGE(
            t->geom.y[i] >= bmin_y - 0.01 && t->geom.y[i] <= bmax_y + 0.01,
            "Clipped vertex y outside buffered tile bounds");
    }

    collector_free(&c);
}

/* Line that completely crosses through a tile from one side to the other. */
static void test_line_encloses_tile_span(void) {
    arpt_geom g = {0};
    g.type = 2;
    /* A horizontal line at lat=10 from lon=-20 to lon=50.
     * At z=3, this crosses multiple tiles end to end.
     * Tile (8, 4) = lon [0, 22.5], lat [0, 22.5] — the line passes through. */
    double x[] = {-20.0, 50.0};
    double y[] = {10.0, 10.0};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Tile (3, 8, 4) should get a clipped segment */
    tile_result *t = find_result(&c, 3, 8, 4);
    TEST_ASSERT_NOT_NULL_MESSAGE(t,
        "Line spanning through tile should produce a segment");
    TEST_ASSERT_TRUE(t->geom.n_coords >= 2);

    /* Both endpoints should be at the buffered tile x-bounds */
    double buf = 22.5 * 8.0 / 256.0;
    double bmin_x = 0.0 - buf, bmax_x = 22.5 + buf;
    double min_x = t->geom.x[0];
    double max_x = t->geom.x[0];
    for (uint32_t i = 1; i < t->geom.n_coords; i++) {
        if (t->geom.x[i] < min_x) min_x = t->geom.x[i];
        if (t->geom.x[i] > max_x) max_x = t->geom.x[i];
    }
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(0.1, bmin_x, min_x,
        "Clipped line should start near buffered tile left edge");
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(0.1, bmax_x, max_x,
        "Clipped line should end near buffered tile right edge");

    collector_free(&c);
}

/* Polygon with hole: exterior encloses tile, hole is far away.
 * Tile should get only the exterior ring (clipped to rectangle), no hole. */
static void test_polygon_encloses_tile_hole_outside(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Exterior: lon [-10, 35], lat [-10, 35] — encloses tile (3,8,4).
     * Hole: lon [25, 30], lat [25, 30] — entirely outside tile (3,8,4)
     * (tile is [0, 22.5] × [0, 22.5], buffered ~[-0.7, 23.2]). */
    double x[] = {
        /* Exterior CCW */
        -10.0, 35.0, 35.0, -10.0, -10.0,
        /* Hole CW — far corner, outside the tile */
        25.0, 25.0, 30.0, 30.0, 25.0
    };
    double y[] = {
        -10.0, -10.0, 35.0, 35.0, -10.0,
        25.0, 30.0, 30.0, 25.0, 25.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Tile (3, 8, 4) should get geometry — only 1 ring (exterior),
     * because the hole is entirely outside this tile. */
    tile_result *t = find_result(&c, 3, 8, 4);
    TEST_ASSERT_NOT_NULL_MESSAGE(t,
        "Tile enclosed by exterior should get geometry");
    TEST_ASSERT_TRUE(t->geom.n_coords >= 4);

    /* Should have only 1 ring (hole clipped away) */
    uint32_t n_rings = 1;
    if (t->geom.offsets && t->geom.n_offsets > 1)
        n_rings = t->geom.n_offsets - 1;
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, n_rings,
        "Tile far from hole should only get exterior ring");

    collector_free(&c);
}

/* Polygon with hole: both exterior AND hole completely enclose the tile.
 * The tile is entirely inside the hole → should get NO geometry. */
static void test_polygon_encloses_tile_hole_also_encloses(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Exterior: lon [-20, 50], lat [-20, 50]
     * Hole: lon [-5, 30], lat [-5, 30]
     * Both enclose tile (3, 8, 4) = [0, 22.5] × [0, 22.5].
     * The tile is entirely within the hole, so no visible polygon. */
    double x[] = {
        /* Exterior CCW */
        -20.0, 50.0, 50.0, -20.0, -20.0,
        /* Hole CW — also encloses the tile */
        -5.0, -5.0, 30.0, 30.0, -5.0
    };
    double y[] = {
        -20.0, -20.0, 50.0, 50.0, -20.0,
        -5.0, 30.0, 30.0, -5.0, -5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Tile (3, 8, 4) is entirely inside the hole.
     * Current slab clipper clips rings independently, so it WILL produce
     * both rings clipped to the tile rectangle. The net area should be
     * near zero (exterior rect - hole rect ≈ 0). */
    tile_result *t = find_result(&c, 3, 8, 4);
    if (t) {
        /* If we get geometry, verify the net area is near zero:
         * exterior and hole both clip to the same rectangle. */
        double total_area = 0.0;
        if (t->geom.offsets && t->geom.n_offsets > 1) {
            uint32_t nr = t->geom.n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = t->geom.offsets[ri];
                uint32_t re = t->geom.offsets[ri + 1];
                total_area += ring_signed_area(t->geom.x + rs, t->geom.y + rs,
                                                re - rs);
            }
        } else {
            total_area = ring_signed_area(t->geom.x, t->geom.y,
                                           t->geom.n_coords);
        }
        double abs_net = total_area < 0 ? -total_area : total_area;
        fprintf(stderr,
            "  Hole-encloses-tile: net area=%.4f (should be ~0)\n", abs_net);
        /* Net area should be near zero since exterior and hole
         * both clip to the same buffered tile rectangle. */
        TEST_ASSERT_TRUE_MESSAGE(abs_net < 1.0,
            "Tile inside hole should have near-zero net area");
    }
    /* If t is NULL, that's also correct — no geometry for this tile. */

    collector_free(&c);
}

/* Polygon encloses tile, hole partially overlaps the tile.
 * Tile should get exterior rectangle + partial hole. */
static void test_polygon_encloses_tile_hole_partial(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Exterior: lon [-10, 35], lat [-10, 35] — fully encloses tile (3,8,4).
     * Hole: lon [10, 30], lat [10, 30] — partially overlaps the tile.
     * Tile (3, 8, 4) = [0, 22.5] × [0, 22.5], buffered ~[-0.7, 23.2].
     * Hole enters the tile from the right side. */
    double x[] = {
        /* Exterior CCW */
        -10.0, 35.0, 35.0, -10.0, -10.0,
        /* Hole CW — overlaps right portion of tile */
        10.0, 10.0, 30.0, 30.0, 10.0
    };
    double y[] = {
        -10.0, -10.0, 35.0, 35.0, -10.0,
        10.0, 30.0, 30.0, 10.0, 10.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    tile_result *t = find_result(&c, 3, 8, 4);
    TEST_ASSERT_NOT_NULL_MESSAGE(t,
        "Tile with partial hole overlap should get geometry");

    /* Should have 2 rings: exterior (full tile rect) + hole (partial) */
    uint32_t n_rings = 1;
    if (t->geom.offsets && t->geom.n_offsets > 1)
        n_rings = t->geom.n_offsets - 1;
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, n_rings,
        "Tile should have exterior + partial hole ring");

    /* Net area should be less than the full tile area but positive.
     * Full tile area ≈ 571 sq.deg (buffered). Hole removes part of it. */
    double total_area = 0.0;
    uint32_t nr = t->geom.n_offsets - 1;
    for (uint32_t ri = 0; ri < nr; ri++) {
        uint32_t rs = t->geom.offsets[ri];
        uint32_t re = t->geom.offsets[ri + 1];
        total_area += ring_signed_area(t->geom.x + rs, t->geom.y + rs,
                                        re - rs);
    }
    double abs_net = total_area < 0 ? -total_area : total_area;
    fprintf(stderr,
        "  Partial hole: net area=%.1f (should be between 100 and 600)\n",
        abs_net);
    TEST_ASSERT_TRUE_MESSAGE(abs_net > 50.0,
        "Net area should be positive — hole only covers part of tile");
    TEST_ASSERT_TRUE_MESSAGE(abs_net < 600.0,
        "Net area should be less than full tile");

    /* Both rings should be closed */
    for (uint32_t ri = 0; ri < nr; ri++) {
        uint32_t rs = t->geom.offsets[ri];
        uint32_t re = t->geom.offsets[ri + 1];
        TEST_ASSERT_TRUE(re - rs >= 4);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, t->geom.x[rs], t->geom.x[re - 1]);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, t->geom.y[rs], t->geom.y[re - 1]);
    }

    collector_free(&c);
}

/* MultiPolygon with hole+island where everything encloses the tile.
 * Exterior encloses tile, hole encloses tile, island encloses tile.
 * Net: exterior - hole + island ≈ same as island alone ≈ tile rect. */
static void test_multipolygon_all_enclose_tile(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Part 0: Huge exterior + huge hole (both enclose tile)
     * Part 1: Huge island (also encloses tile)
     * All three rings clip to the same tile rectangle.
     * Net area = rect - rect + rect = rect. */
    double x[] = {
        /* Part 0, Exterior CCW */
        -30.0, 60.0, 60.0, -30.0, -30.0,
        /* Part 0, Hole CW */
        -10.0, -10.0, 40.0, 40.0, -10.0,
        /* Part 1, Island CCW */
        -5.0, 30.0, 30.0, -5.0, -5.0
    };
    double y[] = {
        -30.0, -30.0, 60.0, 60.0, -30.0,
        -10.0, 40.0, 40.0, -10.0, -10.0,
        -5.0, -5.0, 30.0, 30.0, -5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 5, 10, 15};
    g.offsets = offsets;
    g.n_offsets = 4;
    /* parts removed: multi-types flattened at parse time */

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Tile (3, 8, 4) should get 2 callbacks: part 0 (ext+hole) and part 1 (island).
     * Part 0 net area ≈ 0 (ext and hole both clip to same rect).
     * Part 1 area ≈ tile rect area. */
    int count_for_tile = 0;
    double grand_total = 0.0;
    for (int i = 0; i < c.count; i++) {
        if (c.results[i].z == 3 && c.results[i].x == 8 && c.results[i].y == 4) {
            count_for_tile++;
            arpt_geom *cg = &c.results[i].geom;
            if (cg->offsets && cg->n_offsets > 1) {
                uint32_t nr = cg->n_offsets - 1;
                for (uint32_t ri = 0; ri < nr; ri++) {
                    uint32_t rs = cg->offsets[ri];
                    uint32_t re = cg->offsets[ri + 1];
                    grand_total += ring_signed_area(
                        cg->x + rs, cg->y + rs, re - rs);
                }
            } else {
                grand_total += ring_signed_area(cg->x, cg->y, cg->n_coords);
            }

            /* All rings should be closed */
            if (cg->offsets && cg->n_offsets > 1) {
                uint32_t nr = cg->n_offsets - 1;
                for (uint32_t ri = 0; ri < nr; ri++) {
                    uint32_t rs = cg->offsets[ri];
                    uint32_t re = cg->offsets[ri + 1];
                    TEST_ASSERT_TRUE(re - rs >= 4);
                    TEST_ASSERT_DOUBLE_WITHIN(1e-12,
                        cg->x[rs], cg->x[re - 1]);
                    TEST_ASSERT_DOUBLE_WITHIN(1e-12,
                        cg->y[rs], cg->y[re - 1]);
                }
            }
        }
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, count_for_tile,
        "Flattened polygon should produce 1 callback with all rings");

    /* Grand total ≈ one tile rect area (part 0 cancels out, part 1 adds it back).
     * Tile rect area ≈ 23.9° × 23.9° ≈ 571 sq.deg. */
    double abs_grand = grand_total < 0 ? -grand_total : grand_total;
    fprintf(stderr,
        "  All-enclose: grand total area=%.1f (expected ~571)\n", abs_grand);
    TEST_ASSERT_TRUE_MESSAGE(abs_grand > 300.0,
        "Grand total should approximate one tile rect");
    TEST_ASSERT_TRUE_MESSAGE(abs_grand < 800.0,
        "Grand total should not exceed tile rect significantly");

    collector_free(&c);
}

/* ---- Ring closure stress tests ---- */

/* Strict ring validation: checks every ring in every callback result.
 * - first == last (exact equality)
 * - at least 4 vertices (3 unique + closing)
 * - no consecutive duplicate vertices (except first==last)
 * - non-zero area */
static void assert_all_rings_valid(tile_collector *c, const char *label) {
    char msg[256];
    for (int i = 0; i < c->count; i++) {
        arpt_geom *cg = &c->results[i].geom;
        int z = c->results[i].z, tx = c->results[i].x, ty = c->results[i].y;

        if (cg->offsets && cg->n_offsets > 1) {
            uint32_t nr = cg->n_offsets - 1;
            for (uint32_t ri = 0; ri < nr; ri++) {
                uint32_t rs = cg->offsets[ri];
                uint32_t re = cg->offsets[ri + 1];
                uint32_t rn = re - rs;

                snprintf(msg, sizeof(msg),
                    "%s: tile(%d,%d,%d) ring %u has %u verts (need >= 4)",
                    label, z, tx, ty, ri, rn);
                TEST_ASSERT_TRUE_MESSAGE(rn >= 4, msg);

                /* first == last (exact) */
                snprintf(msg, sizeof(msg),
                    "%s: tile(%d,%d,%d) ring %u not closed: "
                    "first(%.6f,%.6f) != last(%.6f,%.6f)",
                    label, z, tx, ty, ri,
                    cg->x[rs], cg->y[rs], cg->x[re-1], cg->y[re-1]);
                TEST_ASSERT_EQUAL_DOUBLE_MESSAGE(cg->x[rs], cg->x[re-1], msg);
                TEST_ASSERT_EQUAL_DOUBLE_MESSAGE(cg->y[rs], cg->y[re-1], msg);

                /* No consecutive duplicates (except closing vertex) */
                for (uint32_t j = rs; j + 1 < re - 1; j++) {
                    if (cg->x[j] == cg->x[j+1] && cg->y[j] == cg->y[j+1]) {
                        snprintf(msg, sizeof(msg),
                            "%s: tile(%d,%d,%d) ring %u dup at [%u]"
                            " (%.6f,%.6f)",
                            label, z, tx, ty, ri, j,
                            cg->x[j], cg->y[j]);
                        TEST_FAIL_MESSAGE(msg);
                    }
                }

                /* Non-zero area */
                double area = ring_signed_area(cg->x + rs, cg->y + rs, rn);
                double abs_area = area < 0 ? -area : area;
                snprintf(msg, sizeof(msg),
                    "%s: tile(%d,%d,%d) ring %u has zero area",
                    label, z, tx, ty, ri);
                TEST_ASSERT_TRUE_MESSAGE(abs_area > 1e-12, msg);
            }
        } else {
            /* Single ring (no offsets) */
            uint32_t rn = cg->n_coords;
            snprintf(msg, sizeof(msg),
                "%s: tile(%d,%d,%d) single ring has %u verts (need >= 4)",
                label, z, tx, ty, rn);
            TEST_ASSERT_TRUE_MESSAGE(rn >= 4, msg);

            snprintf(msg, sizeof(msg),
                "%s: tile(%d,%d,%d) single ring not closed: "
                "first(%.6f,%.6f) != last(%.6f,%.6f)",
                label, z, tx, ty,
                cg->x[0], cg->y[0], cg->x[rn-1], cg->y[rn-1]);
            TEST_ASSERT_EQUAL_DOUBLE_MESSAGE(cg->x[0], cg->x[rn-1], msg);
            TEST_ASSERT_EQUAL_DOUBLE_MESSAGE(cg->y[0], cg->y[rn-1], msg);

            for (uint32_t j = 0; j + 1 < rn - 1; j++) {
                if (cg->x[j] == cg->x[j+1] && cg->y[j] == cg->y[j+1]) {
                    snprintf(msg, sizeof(msg),
                        "%s: tile(%d,%d,%d) single ring dup at [%u]"
                        " (%.6f,%.6f)",
                        label, z, tx, ty, j, cg->x[j], cg->y[j]);
                    TEST_FAIL_MESSAGE(msg);
                }
            }

            double area = ring_signed_area(cg->x, cg->y, rn);
            double abs_area = area < 0 ? -area : area;
            snprintf(msg, sizeof(msg),
                "%s: tile(%d,%d,%d) single ring has zero area",
                label, z, tx, ty);
            TEST_ASSERT_TRUE_MESSAGE(abs_area > 1e-12, msg);
        }
    }
}

/* 1. Polygon with a vertex exactly on a tile boundary.
 *    At z=3 (16 cols × 8 rows), tile boundaries at lon ..., 0, 22.5, 45, ...
 *    Place a vertex exactly at lon=22.5 (tile right edge). */
static void test_closure_vertex_on_tile_edge(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Triangle with one vertex exactly on the tile boundary lon=22.5.
     * Other two vertices inside tile (8, 4) = [0, 22.5] × [0, 22.5]. */
    double x[] = {5.0, 22.5, 5.0, 5.0};
    double y[] = {5.0, 11.25, 18.0, 5.0};
    g.x = x;
    g.y = y;
    g.n_coords = 4;
    uint32_t offsets[] = {0, 4};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "vertex_on_edge");
    collector_free(&c);
}

/* 2. Polygon with an edge running exactly along a tile boundary.
 *    The right edge of the polygon coincides with the tile boundary. */
static void test_closure_edge_along_tile_boundary(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Rectangle with right edge at lon=22.5 (tile boundary at z=3).
     * Tile (8, 4) = [0, 22.5]. */
    double x[] = {5.0, 22.5, 22.5, 5.0, 5.0};
    double y[] = {5.0, 5.0, 18.0, 18.0, 5.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "edge_along_boundary");
    collector_free(&c);
}

/* 3. Polygon with edges along BOTH tile boundaries (top and right).
 *    Vertex sits exactly at the tile corner. */
static void test_closure_vertex_on_tile_corner(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Triangle: one vertex at the top-right corner of tile (8, 4).
     * Tile (8, 4) = [0, 22.5] × [0, 22.5].
     * Corner is at (22.5, 22.5). */
    double x[] = {5.0, 22.5, 5.0, 5.0};
    double y[] = {5.0, 22.5, 18.0, 5.0};
    g.x = x;
    g.y = y;
    g.n_coords = 4;
    uint32_t offsets[] = {0, 4};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "vertex_on_corner");
    collector_free(&c);
}

/* 4. Diamond crossing all 4 tile edges.
 *    The two-pass slab clipping must produce a proper octagon. */
static void test_closure_diamond_crosses_all_edges(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* At z=3, tile (8, 4) = [0, 22.5] × [0, 22.5].
     * Diamond centered at (11.25, 11.25) with radius 15°,
     * extending beyond all 4 tile edges. */
    double cx = 11.25, cy = 11.25, r = 15.0;
    double x[] = {cx, cx + r, cx, cx - r, cx};
    double y[] = {cy - r, cy, cy + r, cy, cy - r};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* Should produce results in multiple tiles (diamond extends beyond all edges) */
    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "diamond_all_edges");

    /* The center tile should have an octagonal ring (8 vertices + closing = 9) */
    tile_result *center = find_result(&c, 3, 8, 4);
    TEST_ASSERT_NOT_NULL(center);
    /* Diamond clipped to rect should produce more vertices than the original 4 */
    TEST_ASSERT_TRUE_MESSAGE(center->geom.n_coords >= 5,
        "Diamond clipped to rect should gain vertices at intersections");

    collector_free(&c);
}

/* 5. Thin sliver polygon just clipping a tile corner.
 *    This tests near-degenerate ring closure. */
static void test_closure_thin_sliver_at_corner(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* At z=3, tile (8, 4) = [0, 22.5] × [0, 22.5].
     * Buffer ≈ 0.703°. Buffered bounds ≈ [-0.703, 23.203].
     * Thin triangle barely clipping the top-right corner.
     * Vertices: (20, 25), (25, 20), (25, 25) — only the bottom-left
     * part of this triangle overlaps the buffered tile bounds. */
    double x[] = {20.0, 25.0, 25.0, 20.0};
    double y[] = {25.0, 20.0, 25.0, 25.0};
    g.x = x;
    g.y = y;
    g.n_coords = 4;
    uint32_t offsets[] = {0, 4};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* The triangle is small and near a corner — it might clip to a tiny
     * sliver or be rejected as degenerate. Either outcome is acceptable
     * but if it produces output, the rings must be valid. */
    if (c.count > 0) {
        assert_all_rings_valid(&c, "sliver_corner");
    }
    collector_free(&c);
}

/* 6. Long thin polygon at a diagonal angle crossing a tile boundary.
 *    The clip produces a thin quadrilateral whose closure edge
 *    goes between different clip edges. */
static void test_closure_diagonal_crossing(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* A thin polygon at ~45° crossing the right edge of tile (8,4).
     * Tile right edge at lon=22.5.
     * The polygon goes from inside the tile to outside diagonally.
     * The two long edges have slightly different slopes so the clip
     * doesn't degenerate to a collinear set of points. */
    double x[] = {15.0, 30.0, 31.0, 16.0, 15.0};
    double y[] = {5.0, 20.0, 22.0, 7.0, 5.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "diagonal_crossing");
    collector_free(&c);
}

/* 6b. Parallelogram with exactly parallel edges clips to a degenerate
 *     collinear ring — the clipper should correctly discard it. */
static void test_closure_degenerate_parallel_clip(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Both long edges have slope 1, so clipping at a vertical line
     * produces two intersection points at the same (x,y). After dedup,
     * the result is collinear (zero area) and should be discarded. */
    double x[] = {15.0, 30.0, 31.0, 16.0, 15.0};
    double y[] = {5.0, 20.0, 21.0, 6.0, 5.0};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* The tile (8, 4) clip should NOT produce a degenerate polygon.
     * It should either produce a valid ring or nothing. */
    for (int i = 0; i < c.count; i++) {
        if (c.results[i].z == 3 && c.results[i].x == 8 && c.results[i].y == 4) {
            arpt_geom *cg = &c.results[i].geom;
            /* If it produced output, it must be a valid ring */
            TEST_ASSERT_TRUE(cg->n_coords >= 4);
            double area = ring_signed_area(cg->x, cg->y, cg->n_coords);
            double abs_area = area < 0 ? -area : area;
            TEST_ASSERT_TRUE_MESSAGE(abs_area > 1e-12,
                "Degenerate collinear ring should have been discarded");
        }
    }
    /* If no result for that tile, the degenerate ring was correctly discarded */

    collector_free(&c);
}

/* 7. L-shaped concave polygon straddling a tile corner.
 *    The polygon extends beyond both the top and right edges.
 *    The x-slab closing edge will cross the y-slab boundary. */
static void test_closure_l_shape_at_corner(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* At z=3, tile (8, 4) top-right corner is (22.5, 22.5).
     * L-shape: a vertical bar extending above the tile, plus a horizontal
     * bar extending to the right of the tile.
     *
     *        ┌──┐
     *        │  │  ← extends above tile
     *   ─────┘  │
     *   │       │
     *   │       │──────┐
     *   │              │  ← extends right of tile
     *   └──────────────┘
     *
     * Vertices (CCW): */
    double x[] = {
        10.0, 30.0, 30.0, 20.0, 20.0, 15.0, 15.0, 10.0, 10.0
    };
    double y[] = {
        5.0, 5.0, 15.0, 15.0, 30.0, 30.0, 15.0, 15.0, 5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 9;
    uint32_t offsets[] = {0, 9};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "l_shape_corner");

    /* The tile containing the corner (8, 4) should have a valid L-shaped
     * clipped polygon. */
    tile_result *t = find_result(&c, 3, 8, 4);
    TEST_ASSERT_NOT_NULL(t);
    /* L-shape clipped to rect: both arms are clipped to tile bounds.
     * Should produce a concave polygon with more vertices than the
     * original 8 (due to clip intersections). */
    TEST_ASSERT_TRUE(t->geom.n_coords >= 5);

    collector_free(&c);
}

/* 8. T-shaped polygon: horizontal bar crosses the tile, vertical bar
 *    goes above the tile. Tests the two-pass interaction when the
 *    x-slab pass produces a closing edge that the y-slab must clip. */
static void test_closure_t_shape_crossing(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* T-shape:
     *          ┌──┐
     *          │  │  ← vertical bar above tile
     *     ┌────┘  └────┐
     *     │             │  ← horizontal bar inside tile
     *     └─────────────┘
     *
     * At z=3, tile (8, 4) = [0, 22.5] × [0, 22.5].
     * Horizontal bar: lon [-5, 28], lat [5, 12] — crosses left & right.
     * Vertical bar: lon [8, 15], lat [12, 30] — extends above. */
    double x[] = {
        -5.0, 28.0, 28.0, 15.0, 15.0, 8.0, 8.0, -5.0, -5.0
    };
    double y[] = {
        5.0, 5.0, 12.0, 12.0, 30.0, 30.0, 12.0, 12.0, 5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 9;
    uint32_t offsets[] = {0, 9};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "t_shape");
    collector_free(&c);
}

/* 9. Polygon that exits and re-enters the SAME slab boundary at
 *    very close positions. The closing edge between these nearby
 *    points is where artifacts can appear. */
static void test_closure_narrow_reentry(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* A polygon that pokes just barely outside the top of tile (8, 4).
     * It exits at lon=10 going above, and re-enters at lon=13.
     * The excursion is only ~1° above the tile boundary.
     *
     * At z=3, tile top is lat=22.5, buffered top ≈ 23.2.
     * Exit and re-entry on the buffered top edge: */
    double x[] = {
        5.0, 10.0, 11.5, 13.0, 18.0, 18.0, 5.0, 5.0
    };
    double y[] = {
        10.0, 10.0, 25.0, 10.0, 10.0, 20.0, 20.0, 10.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 8;
    uint32_t offsets[] = {0, 8};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "narrow_reentry");
    collector_free(&c);
}

/* 10. Polygon that exits the tile through the right edge and re-enters
 *     through the top edge. The two-pass clipping creates a synthetic
 *     closing edge from x-slab pass that the y-slab must then clip. */
static void test_closure_exit_right_enter_top(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* At z=3, tile (8, 4) = [0, 22.5] × [0, 22.5].
     * Polygon goes:
     *   inside → exits right edge → goes above-right → enters from top
     *   → back inside
     *
     * The x-slab pass clips the portion that's outside the right edge,
     * connecting the exit point to the re-entry with a closing edge
     * along x=buffered_right. But the re-entry comes from ABOVE,
     * so the closing edge goes up beyond the tile top, and the y-slab
     * must clip it. */
    double x[] = {
        5.0, 5.0, 30.0, 30.0, 15.0, 15.0, 5.0
    };
    double y[] = {
        5.0, 15.0, 15.0, 30.0, 30.0, 5.0, 5.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 7;
    uint32_t offsets[] = {0, 7};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "exit_right_enter_top");
    collector_free(&c);
}

/* 11. Same as above but for all four corner transitions:
 *     exit-right/enter-top, exit-top/enter-left,
 *     exit-left/enter-bottom, exit-bottom/enter-right. */
static void test_closure_all_corner_transitions(void) {
    /* A polygon that wraps around the outside of the tile corner.
     * It starts inside, exits bottom, goes around the bottom-right
     * corner outside, and enters from the right. This tests the
     * x-slab/y-slab interaction at every corner combination.
     *
     * Use a polygon that goes around the OUTSIDE of the tile in a
     * clockwise direction (but CCW as a ring), visiting all four corners.
     *
     * At z=3, tile (8, 4) center is (11.25, 11.25).
     * Polygon: a star-like shape with 4 arms extending beyond each edge. */
    arpt_geom g = {0};
    g.type = 3;
    /* 4-pointed star centered on tile (8,4). Each arm extends ~5° beyond
     * the tile boundary in one direction. */
    double cx = 11.25, cy = 11.25;
    double x[] = {
        cx, cx + 5, cx + 18, cx + 5,  /* right arm */
        cx, cx + 5, cx, cx - 5,       /* top arm */
        cx - 18, cx - 5, cx, cx - 5,  /* left arm */
        cx, cx + 5,                    /* bottom arm — close */
        cx                             /* closing vertex */
    };
    double y[] = {
        cy - 18, cy - 5, cy, cy + 5,  /* bottom arm & right arm */
        cy + 18, cy + 5, cy, cy + 5,  /* top arm */
        cy, cy - 5, cy - 18, cy - 5,  /* left arm & bottom */
        cy - 18, cy - 5,              /* closing approach */
        cy - 18                        /* closing vertex */
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 15};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "corner_transitions");
    collector_free(&c);
}

/* 12. Multiple zoom levels: same polygon clipped at z=2 through z=7.
 *     At each zoom the tile grid changes, creating different clip
 *     configurations. All must produce valid closed rings. */
static void test_closure_across_zoom_levels(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* An irregular pentagon that will intersect tile boundaries differently
     * at each zoom level. */
    double x[] = {2.0, 18.0, 22.0, 12.0, -3.0, 2.0};
    double y[] = {3.0, 1.0, 15.0, 24.0, 14.0, 3.0};
    g.x = x;
    g.y = y;
    g.n_coords = 6;
    uint32_t offsets[] = {0, 6};
    g.offsets = offsets;
    g.n_offsets = 2;

    for (int z = 2; z <= 7; z++) {
        tile_collector c;
        collector_init(&c);
        arpt_assign_tiles(&g, &g, z, collect_cb, &c);

        char label[64];
        snprintf(label, sizeof(label), "zoom_%d", z);
        TEST_ASSERT_TRUE(c.count >= 1);
        assert_all_rings_valid(&c, label);
        collector_free(&c);
    }
}

/* 13. Polygon with hole: both rings cross the tile boundary.
 *     Strict validation of closure for every ring at every tile. */
static void test_closure_hole_crossing_boundary(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* At z=3, tile boundary at lon=22.5.
     * Exterior: lon [10, 35], lat [5, 20] — crosses right edge.
     * Hole: lon [15, 30], lat [8, 17] — also crosses right edge. */
    double x[] = {
        /* Exterior CCW */
        10.0, 35.0, 35.0, 10.0, 10.0,
        /* Hole CW */
        15.0, 15.0, 30.0, 30.0, 15.0
    };
    double y[] = {
        5.0, 5.0, 20.0, 20.0, 5.0,
        8.0, 17.0, 17.0, 8.0, 8.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "hole_crossing_boundary");
    collector_free(&c);
}

/* 14. Polygon with hole where the hole ring crosses the boundary
 *     but the exterior doesn't. The exterior clips to the tile rect;
 *     the hole clips to a partial shape. */
static void test_closure_hole_crosses_exterior_doesnt(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* Exterior is entirely within tile (8, 4) = [0, 22.5] × [0, 22.5].
     * Hole extends beyond the right edge at lon=22.5. */
    double x[] = {
        /* Exterior CCW — within tile */
        2.0, 21.0, 21.0, 2.0, 2.0,
        /* Hole CW — extends beyond right edge */
        10.0, 10.0, 28.0, 28.0, 10.0
    };
    double y[] = {
        2.0, 2.0, 21.0, 21.0, 2.0,
        8.0, 15.0, 15.0, 8.0, 8.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 10;
    uint32_t offsets[] = {0, 5, 10};
    g.offsets = offsets;
    g.n_offsets = 3;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "hole_crosses_ext_doesnt");
    collector_free(&c);
}

/* 15. MultiPolygon with hole+island where all three rings cross
 *     the same tile boundary. */
static void test_closure_multipolygon_all_rings_cross(void) {
    arpt_geom g = {0};
    g.type = 3;
    /* All rings cross the right edge of tile (8, 4) at lon=22.5.
     * Part 0: Exterior [5, 30] × [2, 21], Hole [12, 28] × [6, 18]
     * Part 1: Island [15, 25] × [9, 15] */
    double x[] = {
        /* Part 0, Exterior CCW */
        5.0, 30.0, 30.0, 5.0, 5.0,
        /* Part 0, Hole CW */
        12.0, 12.0, 28.0, 28.0, 12.0,
        /* Part 1, Island CCW */
        15.0, 25.0, 25.0, 15.0, 15.0
    };
    double y[] = {
        2.0, 2.0, 21.0, 21.0, 2.0,
        6.0, 18.0, 18.0, 6.0, 6.0,
        9.0, 9.0, 15.0, 15.0, 9.0
    };
    g.x = x;
    g.y = y;
    g.n_coords = 15;
    uint32_t offsets[] = {0, 5, 10, 15};
    g.offsets = offsets;
    g.n_offsets = 4;
    /* parts removed: multi-types flattened at parse time */

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    TEST_ASSERT_TRUE(c.count >= 1);
    assert_all_rings_valid(&c, "multipolygon_all_cross");
    collector_free(&c);
}

/* ---- Regression: simplify-after-clip removes boundary corners ----
 *
 * This documents the root cause of the z=4 wedge artifact: Douglas-Peucker
 * simplification applied AFTER clipping can remove tile-boundary corner
 * vertices, creating diagonal lines across tile interiors.
 *
 * The fix is to simplify BEFORE clipping (in the pipeline), so this test
 * just verifies that the clipper itself produces correct boundary vertices
 * that a subsequent simplifier could remove. */
static void test_regression_boundary_corner_preserved(void) {
    /* A coastline-like polygon that, when clipped to a tile, produces
     * a ring tracing along two tile edges and through a corner.
     *
     * At z=3, tile (8, 4) = [0, 22.5] × [0, 22.5], buffered ~[-0.7, 23.2].
     * The polygon exits the top edge near x=20, goes around the top-right
     * corner (outside the tile), and re-enters the right edge near y=18.
     * The clipped ring should have the corner vertex near (23.2, 22.5). */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {5.0, 20.0, 30.0, 30.0, 25.0, 5.0, 5.0};
    double y[] = {5.0, 5.0, 5.0, 25.0, 25.0, 18.0, 5.0};
    g.x = x;
    g.y = y;
    g.n_coords = 7;
    uint32_t offsets[] = {0, 7};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, &g, 3, collect_cb, &c);

    /* The clipped output should cover the tile — look across all results
     * for tile (3,8,4). With boundary walk, corner vertices are inserted
     * explicitly. Check that at least one ring has a vertex near the
     * top-right corner. */
    double buf = 22.5 * 8.0 / 256.0;
    double corner_x = 22.5 + buf;
    double corner_y = 22.5 + buf;
    bool found_corner = false;
    bool found_tile = false;
    for (int i = 0; i < c.count; i++) {
        if (c.results[i].z != 3 || c.results[i].x != 8 ||
            c.results[i].y != 4) continue;
        found_tile = true;
        arpt_geom *cg = &c.results[i].geom;
        for (uint32_t j = 0; j < cg->n_coords; j++) {
            if (cg->x[j] > 22.0 && cg->y[j] > 22.0) {
                found_corner = true;
                TEST_ASSERT_DOUBLE_WITHIN(1.0, corner_x, cg->x[j]);
                TEST_ASSERT_DOUBLE_WITHIN(1.0, corner_y, cg->y[j]);
            }
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_tile, "Tile (3,8,4) should exist");
    TEST_ASSERT_TRUE_MESSAGE(found_corner,
        "A ring in the tile should have a vertex near the tile corner");

    assert_all_rings_valid(&c, "boundary_corner");
    collector_free(&c);
}

/* When arpt_assign_tiles clips a simplified polygon, interior tiles
 * (fully inside the polygon) produce no clip output and fall back to
 * a point-in-polygon test.  If the simplified boundary shrank relative
 * to the original (corners cut by DP), tile centers near the boundary
 * may land outside the simplified ring but inside the original.
 *
 * The fix: the point-in-polygon fallback uses the original geometry.
 * This test verifies that by passing a manually shrunk "simplified"
 * polygon and a larger "original" — tiles inside the original but
 * outside the simplified must still receive geometry. */
static void test_simplified_polygon_interior_tiles(void) {
    /* Original: large rectangle lon [-50, -10], lat [60, 80]. */
    double orig_x[] = {-50.0, -10.0, -10.0, -50.0, -50.0};
    double orig_y[] = { 60.0,  60.0,  80.0,  80.0,  60.0};
    uint32_t orig_off[] = {0, 5};
    arpt_geom original = {
        .type = 3, .x = orig_x, .y = orig_y, .n_coords = 5,
        .offsets = orig_off, .n_offsets = 2
    };

    /* "Simplified": triangle with same bbox but less interior area.
     * The east boundary is now a diagonal from (-10,70) instead of a
     * vertical edge, so the lower-right and upper-right are cut off. */
    double simp_x[] = {-50.0, -10.0, -50.0, -50.0};
    double simp_y[] = { 60.0,  70.0,  80.0,  60.0};
    uint32_t simp_off[] = {0, 4};
    arpt_geom simplified = {
        .type = 3, .x = simp_x, .y = simp_y, .n_coords = 4,
        .offsets = simp_off, .n_offsets = 2
    };

    /* At zoom 5 (64 cols × 32 rows, tile ≈ 5.625°).
     * Tile (5, 29, 27): lon [-16.875, -11.25], lat [61.875, 67.5].
     * Center ≈ (-14.06, 64.69).
     * - Inside the original rectangle (east boundary at x=-10)
     * - Outside the simplified triangle (diagonal goes from (-50,60)
     *   to (-10,70); at lat 64.69 the boundary is at x≈-31)
     * - The triangle boundary doesn't cross this tile → empty clip
     *   → falls back to point-in-polygon. */
    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&simplified, &original, 5, collect_cb, &c);

    tile_result *t = find_result(&c, 5, 29, 27);
    TEST_ASSERT_NOT_NULL_MESSAGE(t,
        "Tile (5,29,27) inside original but outside simplified must get "
        "geometry via point-in-polygon on the original — fails if the "
        "fallback uses the simplified geometry");
    TEST_ASSERT_TRUE(t->geom.n_coords >= 4);

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
    RUN_TEST(test_polygon_with_hole_within_tile);
    RUN_TEST(test_polygon_with_hole_crossing_tile);
    RUN_TEST(test_polygon_hole_empty_interior);
    RUN_TEST(test_polygon_hole_only_frame);
    RUN_TEST(test_multipolygon_hole_and_island_within_tile);
    RUN_TEST(test_multipolygon_hole_and_island_crossing_tile);
    RUN_TEST(test_multipolygon_hole_island_high_zoom);
    RUN_TEST(test_multipolygon_hole_island_area_accounting);
    RUN_TEST(test_polygon_encloses_tile_completely);
    RUN_TEST(test_line_encloses_tile_span);
    RUN_TEST(test_polygon_encloses_tile_hole_outside);
    RUN_TEST(test_polygon_encloses_tile_hole_also_encloses);
    RUN_TEST(test_polygon_encloses_tile_hole_partial);
    RUN_TEST(test_multipolygon_all_enclose_tile);
    RUN_TEST(test_closure_vertex_on_tile_edge);
    RUN_TEST(test_closure_edge_along_tile_boundary);
    RUN_TEST(test_closure_vertex_on_tile_corner);
    RUN_TEST(test_closure_diamond_crosses_all_edges);
    RUN_TEST(test_closure_thin_sliver_at_corner);
    RUN_TEST(test_closure_diagonal_crossing);
    RUN_TEST(test_closure_degenerate_parallel_clip);
    RUN_TEST(test_closure_l_shape_at_corner);
    RUN_TEST(test_closure_t_shape_crossing);
    RUN_TEST(test_closure_narrow_reentry);
    RUN_TEST(test_closure_exit_right_enter_top);
    RUN_TEST(test_closure_all_corner_transitions);
    RUN_TEST(test_closure_across_zoom_levels);
    RUN_TEST(test_closure_hole_crossing_boundary);
    RUN_TEST(test_closure_hole_crosses_exterior_doesnt);
    RUN_TEST(test_closure_multipolygon_all_rings_cross);
    RUN_TEST(test_regression_boundary_corner_preserved);
    RUN_TEST(test_reentrant_same_edge);
    RUN_TEST(test_reentrant_top_edge);
    RUN_TEST(test_simplified_polygon_interior_tiles);
    return UNITY_END();
}
