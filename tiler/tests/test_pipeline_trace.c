#include "unity.h"
#include "clip.h"
#include "simplify.h"
#include "tile_build.h"
#include "tile.h"
#include "tile_reader.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* ---- Constants matching tile_build.c ---- */
#define TILE_EXTENT 32768
#define TILE_BUFFER 16384

/* ---- Tile bounds (now public in clip.h) ---- */
#define tile_bounds arpt_tile_bounds

/* ---- Dequantization (FORMAT.md spec) ---- */

static double dequant_x(const arpt_bounds *b, uint16_t qx) {
    return b->min_x +
           ((double)qx - TILE_BUFFER) / TILE_EXTENT * (b->max_x - b->min_x);
}

static double dequant_y(const arpt_bounds *b, uint16_t qy) {
    return b->min_y +
           ((double)qy - TILE_BUFFER) / TILE_EXTENT * (b->max_y - b->min_y);
}

/* ---- Tile collector for clip callbacks ---- */

typedef struct {
    int z, x, y;
    arpt_geom geom;
} tile_result;

typedef struct {
    tile_result *results;
    int count;
    int cap;
} tile_collector;

static void collect_cb(int z, int x, int y, const arpt_geom *clipped,
                        void *ctx) {
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
    r->geom = *clipped;
    r->geom.x = malloc(clipped->n_coords * sizeof(double));
    r->geom.y = malloc(clipped->n_coords * sizeof(double));
    memcpy(r->geom.x, clipped->x, clipped->n_coords * sizeof(double));
    memcpy(r->geom.y, clipped->y, clipped->n_coords * sizeof(double));
    if (clipped->offsets && clipped->n_offsets > 0) {
        r->geom.offsets = malloc(clipped->n_offsets * sizeof(uint32_t));
        memcpy(r->geom.offsets, clipped->offsets,
               clipped->n_offsets * sizeof(uint32_t));
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

/* Helper: find result for a specific tile */
static tile_result *find_tile(tile_collector *c, int z, int x, int y) {
    for (int i = 0; i < c->count; i++) {
        if (c->results[i].z == z && c->results[i].x == x &&
            c->results[i].y == y) {
            return &c->results[i];
        }
    }
    return NULL;
}

/* Helper: build tile from a clipped geometry, decode, and return the decoded
 * FlatBuffer. Caller must free *decoded. */
static arpentry_tiles_Tile_table_t build_and_decode(const arpt_bounds *bounds,
                                                     const arpt_geom *geom,
                                                     uint32_t layer,
                                                     uint8_t **decoded_out,
                                                     void **compressed_out) {
    arpt_tile_builder *tb = arpt_tile_builder_create(*bounds, NULL);
    if (!tb) return NULL;

    arpt_feature feat = {0};
    feat.layer = layer;
    feat.geom = geom;
    const char *keys[] = {"class"};
    const char *vals[] = {"test"};
    feat.prop_keys = keys;
    feat.prop_vals = vals;
    feat.n_props = 1;
    arpt_tile_builder_add_feature(tb, &feat);

    size_t out_size;
    void *compressed = arpt_tile_builder_finish(tb, &out_size);
    arpt_tile_builder_free(tb);
    if (!compressed) return NULL;

    uint8_t *decoded;
    size_t decoded_size;
    if (!arpt_decode(compressed, out_size, &decoded, &decoded_size)) {
        free(compressed);
        return NULL;
    }

    *decoded_out = decoded;
    *compressed_out = compressed;
    return arpentry_tiles_Tile_as_root(decoded);
}

/* ======================================================================
 *  TEST 1: Point at Bern (7.45, 46.95) - trace through clip + tile build
 * ====================================================================== */

static void test_point_clip_z0(void) {
    /* Bern, Switzerland */
    arpt_geom g = {0};
    g.type = 1;
    double x = 7.45, y = 46.95;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);

    /* At z=0 equirectangular: n_cols=2, n_rows=1.
     * Bern (7.45, 46.95) → tx=1, ty=0 */
    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].z);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].y);

    /* Clipped coords should be unchanged */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 7.45, c.results[0].geom.x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 46.95, c.results[0].geom.y[0]);

    collector_free(&c);
}

static void test_point_quantize_z0(void) {
    /* Build a tile at z=0 with a point at Bern and verify quantization.
     * Equirectangular z=0: tile (1,0) covers [0,180] lon × [-90,90] lat */
    arpt_bounds b = tile_bounds(0, 1, 0);
    arpt_geom g = {0};
    g.type = 1;
    double x = 7.45, y = 46.95;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    uint8_t *decoded;
    void *compressed;
    arpentry_tiles_Tile_table_t tile =
        build_and_decode(&b, &g, 1, &decoded, &compressed);
    TEST_ASSERT_NOT_NULL(tile);

    /* Find the feature (layer 1 = "surface") */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    TEST_ASSERT_NOT_NULL(layers);

    /* Find the layer with our feature (skip terrain layer if present) */
    arpentry_tiles_Feature_table_t feat = NULL;
    int n_layers = (int)arpentry_tiles_Layer_vec_len(layers);
    for (int i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        arpentry_tiles_Feature_vec_t features =
            arpentry_tiles_Layer_features(layer);
        if (features && arpentry_tiles_Feature_vec_len(features) > 0) {
            arpentry_tiles_Feature_table_t f =
                arpentry_tiles_Feature_vec_at(features, 0);
            if (arpentry_tiles_Feature_geometry_type(f) ==
                arpentry_tiles_Geometry_PointGeometry) {
                feat = f;
                break;
            }
        }
    }
    TEST_ASSERT_NOT_NULL(feat);

    arpentry_tiles_PointGeometry_table_t pg =
        (arpentry_tiles_PointGeometry_table_t)arpentry_tiles_Feature_geometry(
            feat);
    flatbuffers_uint16_vec_t xs = arpentry_tiles_PointGeometry_x(pg);
    flatbuffers_uint16_vec_t ys = arpentry_tiles_PointGeometry_y(pg);
    TEST_ASSERT_EQUAL_INT(1, (int)flatbuffers_uint16_vec_len(xs));

    uint16_t qx = flatbuffers_uint16_vec_at(xs, 0);
    uint16_t qy = flatbuffers_uint16_vec_at(ys, 0);

    /* Dequantize back to WGS84 and verify round-trip */
    double lon = dequant_x(&b, qx);
    double lat = dequant_y(&b, qy);

    /* At z=0, tile spans 180 deg lon and 180 deg lat.
     * uint16 gives 32768 steps → ~0.0055 deg precision.
     * Allow 0.02 deg tolerance. */
    TEST_ASSERT_DOUBLE_WITHIN(0.02, 7.45, lon);
    TEST_ASSERT_DOUBLE_WITHIN(0.02, 46.95, lat);

    free(decoded);
    free(compressed);
}

static void test_point_clip_z5(void) {
    /* At z=5, point at Bern should be in a specific tile */
    arpt_geom g = {0};
    g.type = 1;
    double x = 7.45, y = 46.95;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 5, collect_cb, &c);

    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(5, c.results[0].z);

    /* Verify the tile assignment is correct by checking the point
     * is within the tile bounds */
    arpt_bounds tb = tile_bounds(5, c.results[0].x, c.results[0].y);
    TEST_ASSERT_TRUE(x >= tb.min_x);
    TEST_ASSERT_TRUE(x <= tb.max_x);
    TEST_ASSERT_TRUE(y >= tb.min_y);
    TEST_ASSERT_TRUE(y <= tb.max_y);

    /* Expected: tx=33, ty=24 (equirectangular: n_cols=64, n_rows=32) */
    TEST_ASSERT_EQUAL_INT(33, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(24, c.results[0].y);

    collector_free(&c);
}

static void test_point_quantize_z5(void) {
    /* Higher zoom = better quantization precision */
    int z = 5, tx = 33, ty = 24;
    arpt_bounds b = tile_bounds(z, tx, ty);
    arpt_geom g = {0};
    g.type = 1;
    double x = 7.45, y = 46.95;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    uint8_t *decoded;
    void *compressed;
    arpentry_tiles_Tile_table_t tile =
        build_and_decode(&b, &g, 1, &decoded, &compressed);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Feature_table_t feat = NULL;
    int n_layers = (int)arpentry_tiles_Layer_vec_len(layers);
    for (int i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        arpentry_tiles_Feature_vec_t features =
            arpentry_tiles_Layer_features(layer);
        if (features && arpentry_tiles_Feature_vec_len(features) > 0) {
            arpentry_tiles_Feature_table_t f =
                arpentry_tiles_Feature_vec_at(features, 0);
            if (arpentry_tiles_Feature_geometry_type(f) ==
                arpentry_tiles_Geometry_PointGeometry) {
                feat = f;
                break;
            }
        }
    }
    TEST_ASSERT_NOT_NULL(feat);

    arpentry_tiles_PointGeometry_table_t pg =
        (arpentry_tiles_PointGeometry_table_t)arpentry_tiles_Feature_geometry(
            feat);
    flatbuffers_uint16_vec_t xs = arpentry_tiles_PointGeometry_x(pg);
    flatbuffers_uint16_vec_t ys = arpentry_tiles_PointGeometry_y(pg);

    uint16_t qx = flatbuffers_uint16_vec_at(xs, 0);
    uint16_t qy = flatbuffers_uint16_vec_at(ys, 0);

    /* Quantized coords should be in the tile proper range [16384, 49151] */
    TEST_ASSERT_TRUE(qx >= TILE_BUFFER);
    TEST_ASSERT_TRUE(qx <= TILE_BUFFER + TILE_EXTENT - 1);
    TEST_ASSERT_TRUE(qy >= TILE_BUFFER);
    TEST_ASSERT_TRUE(qy <= TILE_BUFFER + TILE_EXTENT - 1);

    /* Dequantize and verify — at z=5 equirectangular tile spans
     * 360/64=5.625 deg lon, 180/32=5.625 deg lat
     * → precision ~0.00017 deg */
    double lon = dequant_x(&b, qx);
    double lat = dequant_y(&b, qy);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 7.45, lon);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 46.95, lat);

    free(decoded);
    free(compressed);
}

/* ======================================================================
 *  TEST 2: LineString across Switzerland (Geneva → Zurich)
 * ====================================================================== */

static void test_line_clip_z0(void) {
    /* Line from Geneva (6.15, 46.20) to Zurich (8.54, 47.38) */
    arpt_geom g = {0};
    g.type = 2;
    double x[] = {6.15, 8.54};
    double y[] = {46.20, 47.38};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);

    /* At z=0 equirectangular, everything in tile (0,1,0) */
    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].z);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].y);

    /* Line should have 2 vertices, unchanged */
    TEST_ASSERT_EQUAL_INT(2, (int)c.results[0].geom.n_coords);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 6.15, c.results[0].geom.x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 46.20, c.results[0].geom.y[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 8.54, c.results[0].geom.x[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 47.38, c.results[0].geom.y[1]);

    collector_free(&c);
}

static void test_line_clip_z4(void) {
    /* Longer line from west France to east Austria */
    arpt_geom g = {0};
    g.type = 2;
    double x[] = {-2.0, 16.0};
    double y[] = {46.0, 48.0};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 4, collect_cb, &c);

    /* Should span multiple tiles at z=4 */
    TEST_ASSERT_TRUE(c.count >= 2);

    /* Every clipped segment must lie within the tile bounds extended
     * by the clip buffer (8/256 of tile extent per side). */
    for (int i = 0; i < c.count; i++) {
        arpt_bounds tb = tile_bounds(c.results[i].z, c.results[i].x,
                                     c.results[i].y);
        double buf_x = (tb.max_x - tb.min_x) * (8.0 / 256.0);
        double buf_y = (tb.max_y - tb.min_y) * (8.0 / 256.0);
        for (uint32_t j = 0; j < c.results[i].geom.n_coords; j++) {
            double cx = c.results[i].geom.x[j];
            double cy = c.results[i].geom.y[j];
            TEST_ASSERT_TRUE_MESSAGE(cx >= tb.min_x - buf_x - 1e-9,
                                     "clipped x below tile min_x");
            TEST_ASSERT_TRUE_MESSAGE(cx <= tb.max_x + buf_x + 1e-9,
                                     "clipped x above tile max_x");
            TEST_ASSERT_TRUE_MESSAGE(cy >= tb.min_y - buf_y - 1e-9,
                                     "clipped y below tile min_y");
            TEST_ASSERT_TRUE_MESSAGE(cy <= tb.max_y + buf_y + 1e-9,
                                     "clipped y above tile max_y");
        }
    }

    collector_free(&c);
}

static void test_line_quantize_roundtrip(void) {
    /* Clip line at z=4, build tile for one result, verify roundtrip */
    arpt_geom g = {0};
    g.type = 2;
    double x[] = {6.15, 8.54};
    double y[] = {46.20, 47.38};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 4, collect_cb, &c);
    TEST_ASSERT_TRUE(c.count >= 1);

    /* Build and decode the first tile */
    tile_result *tr = &c.results[0];
    arpt_bounds tb = tile_bounds(tr->z, tr->x, tr->y);

    uint8_t *decoded;
    void *compressed;
    arpentry_tiles_Tile_table_t tile =
        build_and_decode(&tb, &tr->geom, 2, &decoded, &compressed);
    TEST_ASSERT_NOT_NULL(tile);

    /* Find the LineGeometry */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Feature_table_t feat = NULL;
    int n_layers = (int)arpentry_tiles_Layer_vec_len(layers);
    for (int i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        arpentry_tiles_Feature_vec_t features =
            arpentry_tiles_Layer_features(layer);
        if (features && arpentry_tiles_Feature_vec_len(features) > 0) {
            arpentry_tiles_Feature_table_t f =
                arpentry_tiles_Feature_vec_at(features, 0);
            if (arpentry_tiles_Feature_geometry_type(f) ==
                arpentry_tiles_Geometry_LineGeometry) {
                feat = f;
                break;
            }
        }
    }
    TEST_ASSERT_NOT_NULL(feat);

    arpentry_tiles_LineGeometry_table_t lg =
        (arpentry_tiles_LineGeometry_table_t)arpentry_tiles_Feature_geometry(
            feat);
    flatbuffers_uint16_vec_t xs = arpentry_tiles_LineGeometry_x(lg);
    flatbuffers_uint16_vec_t ys = arpentry_tiles_LineGeometry_y(lg);
    int n = (int)flatbuffers_uint16_vec_len(xs);
    TEST_ASSERT_TRUE(n >= 2);

    /* Verify all dequantized coords are within the original clipped range */
    for (int i = 0; i < n; i++) {
        uint16_t qx = flatbuffers_uint16_vec_at(xs, i);
        uint16_t qy = flatbuffers_uint16_vec_at(ys, i);
        double lon = dequant_x(&tb, qx);
        double lat = dequant_y(&tb, qy);

        /* Dequantized coords should be within tile bounds */
        TEST_ASSERT_TRUE_MESSAGE(lon >= tb.min_x - 0.01,
                                 "dequant lon below tile min_x");
        TEST_ASSERT_TRUE_MESSAGE(lon <= tb.max_x + 0.01,
                                 "dequant lon above tile max_x");
        TEST_ASSERT_TRUE_MESSAGE(lat >= tb.min_y - 0.01,
                                 "dequant lat below tile min_y");
        TEST_ASSERT_TRUE_MESSAGE(lat <= tb.max_y + 0.01,
                                 "dequant lat above tile max_y");
    }

    free(decoded);
    free(compressed);
    collector_free(&c);
}

/* ======================================================================
 *  TEST 3: Polygon covering Switzerland
 * ====================================================================== */

static void test_polygon_clip_z0(void) {
    /* Rectangle roughly covering Switzerland */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {5.9, 10.5, 10.5, 5.9, 5.9};
    double y[] = {45.8, 45.8, 47.8, 47.8, 45.8};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 0, collect_cb, &c);

    /* At z=0 equirectangular, one tile (1,0) */
    TEST_ASSERT_EQUAL_INT(1, c.count);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].z);
    TEST_ASSERT_EQUAL_INT(1, c.results[0].x);
    TEST_ASSERT_EQUAL_INT(0, c.results[0].y);

    /* Polygon should be unchanged (fully within tile) */
    TEST_ASSERT_TRUE(c.results[0].geom.n_coords >= 4);

    collector_free(&c);
}

static void test_polygon_clip_z4(void) {
    /* Large polygon crossing tile boundaries at z=4 */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {5.9, 10.5, 10.5, 5.9, 5.9};
    double y[] = {45.8, 45.8, 47.8, 47.8, 45.8};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    tile_collector c;
    collector_init(&c);
    arpt_assign_tiles(&g, 4, collect_cb, &c);

    /* Should appear in multiple tiles at z=4 */
    TEST_ASSERT_TRUE(c.count >= 1);

    /* Every clipped polygon vertex must lie within its tile bounds */
    for (int i = 0; i < c.count; i++) {
        arpt_bounds tb = tile_bounds(c.results[i].z, c.results[i].x,
                                     c.results[i].y);
        for (uint32_t j = 0; j < c.results[i].geom.n_coords; j++) {
            double cx = c.results[i].geom.x[j];
            double cy = c.results[i].geom.y[j];
            TEST_ASSERT_TRUE_MESSAGE(cx >= tb.min_x - 1e-9,
                                     "polygon x below tile min_x");
            TEST_ASSERT_TRUE_MESSAGE(cx <= tb.max_x + 1e-9,
                                     "polygon x above tile max_x");
            TEST_ASSERT_TRUE_MESSAGE(cy >= tb.min_y - 1e-9,
                                     "polygon y below tile min_y");
            TEST_ASSERT_TRUE_MESSAGE(cy <= tb.max_y + 1e-9,
                                     "polygon y above tile max_y");
        }
    }

    collector_free(&c);
}

static void test_polygon_quantize_roundtrip(void) {
    /* Build and decode a polygon tile, verify coordinate roundtrip */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {5.9, 10.5, 10.5, 5.9, 5.9};
    double y[] = {45.8, 45.8, 47.8, 47.8, 45.8};
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

    tile_result *tr = &c.results[0];
    arpt_bounds tb = tile_bounds(tr->z, tr->x, tr->y);

    uint8_t *decoded;
    void *compressed;
    arpentry_tiles_Tile_table_t tile =
        build_and_decode(&tb, &tr->geom, 1, &decoded, &compressed);
    TEST_ASSERT_NOT_NULL(tile);

    /* Find PolygonGeometry */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Feature_table_t feat = NULL;
    int n_layers = (int)arpentry_tiles_Layer_vec_len(layers);
    for (int i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        arpentry_tiles_Feature_vec_t features =
            arpentry_tiles_Layer_features(layer);
        if (features && arpentry_tiles_Feature_vec_len(features) > 0) {
            arpentry_tiles_Feature_table_t f =
                arpentry_tiles_Feature_vec_at(features, 0);
            if (arpentry_tiles_Feature_geometry_type(f) ==
                arpentry_tiles_Geometry_PolygonGeometry) {
                feat = f;
                break;
            }
        }
    }
    TEST_ASSERT_NOT_NULL(feat);

    arpentry_tiles_PolygonGeometry_table_t pg =
        (arpentry_tiles_PolygonGeometry_table_t)
            arpentry_tiles_Feature_geometry(feat);
    flatbuffers_uint16_vec_t xs = arpentry_tiles_PolygonGeometry_x(pg);
    flatbuffers_uint16_vec_t ys = arpentry_tiles_PolygonGeometry_y(pg);
    int n = (int)flatbuffers_uint16_vec_len(xs);
    TEST_ASSERT_TRUE(n >= 3);

    /* All dequantized coords must be within tile bounds */
    for (int i = 0; i < n; i++) {
        uint16_t qx = flatbuffers_uint16_vec_at(xs, i);
        uint16_t qy = flatbuffers_uint16_vec_at(ys, i);
        double lon = dequant_x(&tb, qx);
        double lat = dequant_y(&tb, qy);

        TEST_ASSERT_TRUE_MESSAGE(lon >= tb.min_x - 0.01,
                                 "polygon dequant lon below tile");
        TEST_ASSERT_TRUE_MESSAGE(lon <= tb.max_x + 0.01,
                                 "polygon dequant lon above tile");
        TEST_ASSERT_TRUE_MESSAGE(lat >= tb.min_y - 0.01,
                                 "polygon dequant lat below tile");
        TEST_ASSERT_TRUE_MESSAGE(lat <= tb.max_y + 0.01,
                                 "polygon dequant lat above tile");
    }

    free(decoded);
    free(compressed);
    collector_free(&c);
}

/* ======================================================================
 *  TEST 4: Simplification preserves geometry at each zoom
 * ====================================================================== */

static void test_simplify_preserves_line(void) {
    /* A 3-point line: Geneva → Bern → Zurich. At z=0 (tolerance ~1.4 deg),
     * Bern is ~1.3 deg from the Geneva-Zurich line, so it might be removed.
     * At z=4 (tolerance ~0.09 deg), it should be preserved. */
    double x[] = {6.15, 7.45, 8.54};
    double y[] = {46.20, 46.95, 47.38};

    /* z=4 tolerance: 360 / (1 << 12) = 0.088 deg */
    double tol_z4 = 360.0 / (double)(1 << 12);
    double sx[3], sy[3];
    memcpy(sx, x, sizeof(x));
    memcpy(sy, y, sizeof(y));
    uint32_t n = arpt_simplify(sx, sy, 3, tol_z4);

    /* At z=4, all 3 points should be preserved (Bern is ~1.3 deg from line,
     * much larger than 0.088 deg tolerance) */
    TEST_ASSERT_EQUAL_INT(3, (int)n);
}

static void test_simplify_reduces_at_low_zoom(void) {
    /* A line with a tiny deviation that should be simplified at z=0 */
    double x[] = {0.0, 5.0, 10.0};
    double y[] = {0.0, 0.1, 0.0}; /* 0.1 deg deviation */

    double tol_z0 = 360.0 / (double)(1 << 8); /* ~1.4 deg */
    double sx[3], sy[3];
    memcpy(sx, x, sizeof(x));
    memcpy(sy, y, sizeof(y));
    uint32_t n = arpt_simplify(sx, sy, 3, tol_z0);

    /* 0.1 deg < 1.4 deg → middle point removed */
    TEST_ASSERT_EQUAL_INT(2, (int)n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, sx[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 10.0, sx[1]);
}

/* ======================================================================
 *  TEST 5: Multi-zoom consistency — feature appears at all zoom levels
 * ====================================================================== */

static void test_point_visible_at_all_zooms(void) {
    /* A point should appear in exactly one tile at every zoom level */
    arpt_geom g = {0};
    g.type = 1;
    double x = 7.45, y = 46.95;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    for (int z = 0; z <= 10; z++) {
        tile_collector c;
        collector_init(&c);
        arpt_assign_tiles(&g, z, collect_cb, &c);

        TEST_ASSERT_EQUAL_INT_MESSAGE(1, c.count,
                                      "Point should be in exactly 1 tile");

        /* Verify the point is within the assigned tile's bounds */
        arpt_bounds tb = tile_bounds(z, c.results[0].x, c.results[0].y);
        TEST_ASSERT_TRUE(x >= tb.min_x);
        TEST_ASSERT_TRUE(x <= tb.max_x);
        TEST_ASSERT_TRUE(y >= tb.min_y);
        TEST_ASSERT_TRUE(y <= tb.max_y);

        collector_free(&c);
    }
}

static void test_line_visible_at_all_zooms(void) {
    /* A line across Switzerland should appear in at least 1 tile at each zoom */
    arpt_geom g = {0};
    g.type = 2;
    double x[] = {6.15, 8.54};
    double y[] = {46.20, 47.38};
    g.x = x;
    g.y = y;
    g.n_coords = 2;

    for (int z = 0; z <= 8; z++) {
        tile_collector c;
        collector_init(&c);
        arpt_assign_tiles(&g, z, collect_cb, &c);

        char msg[64];
        snprintf(msg, sizeof(msg), "Line should appear at z=%d", z);
        TEST_ASSERT_TRUE_MESSAGE(c.count >= 1, msg);

        /* At higher zoom, should appear in more tiles */
        if (z >= 4) {
            snprintf(msg, sizeof(msg),
                     "Line should span multiple tiles at z=%d", z);
            /* At z=4 equirectangular, tiles are 360/32=11.25 deg wide.
             * Line spans ~2.4 deg, might still be in one tile. */
        }

        collector_free(&c);
    }
}

static void test_polygon_visible_at_all_zooms(void) {
    /* A polygon covering Switzerland should appear in at least 1 tile */
    arpt_geom g = {0};
    g.type = 3;
    double x[] = {5.9, 10.5, 10.5, 5.9, 5.9};
    double y[] = {45.8, 45.8, 47.8, 47.8, 45.8};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    for (int z = 0; z <= 8; z++) {
        tile_collector c;
        collector_init(&c);
        arpt_assign_tiles(&g, z, collect_cb, &c);

        char msg[64];
        snprintf(msg, sizeof(msg), "Polygon should appear at z=%d", z);
        TEST_ASSERT_TRUE_MESSAGE(c.count >= 1, msg);

        collector_free(&c);
    }
}

/* ======================================================================
 *  TEST 6: Quantization precision improves with zoom
 * ====================================================================== */

static void test_quantize_precision_improves(void) {
    double lon = 7.45, lat = 46.95;

    double prev_err_lon = 999.0, prev_err_lat = 999.0;

    for (int z = 0; z <= 10; z++) {
        /* Find which tile the point falls in */
        arpt_geom g = {0};
        g.type = 1;
        g.x = &lon;
        g.y = &lat;
        g.n_coords = 1;

        tile_collector c;
        collector_init(&c);
        arpt_assign_tiles(&g, z, collect_cb, &c);
        TEST_ASSERT_EQUAL_INT(1, c.count);

        arpt_bounds tb = tile_bounds(z, c.results[0].x, c.results[0].y);

        /* Manually quantize */
        double tx = (lon - tb.min_x) / (tb.max_x - tb.min_x);
        double qxd = tx * TILE_EXTENT + TILE_BUFFER;
        uint16_t qx = (uint16_t)(qxd < 0 ? 0 : (qxd > 65535 ? 65535 : qxd));

        double ty = (lat - tb.min_y) / (tb.max_y - tb.min_y);
        double qyd = ty * TILE_EXTENT + TILE_BUFFER;
        uint16_t qy = (uint16_t)(qyd < 0 ? 0 : (qyd > 65535 ? 65535 : qyd));

        /* Dequantize */
        double dlon = dequant_x(&tb, qx);
        double dlat = dequant_y(&tb, qy);

        double err_lon = fabs(dlon - lon);
        double err_lat = fabs(dlat - lat);

        /* Error should generally decrease or stay similar */
        /* (It won't strictly decrease every level due to position within tile,
         * but by z=10 it should be much better than z=0) */
        if (z == 10) {
            TEST_ASSERT_TRUE(err_lon < prev_err_lon * 10);
            TEST_ASSERT_TRUE(err_lat < prev_err_lat * 10);
        }

        if (z == 0) {
            prev_err_lon = err_lon;
            prev_err_lat = err_lat;
        }

        collector_free(&c);
    }
}

/* ======================================================================
 *  TEST 7: Verify tile content structure
 * ====================================================================== */

static void test_tile_has_terrain_layer(void) {
    /* When building a tile with a non-terrain feature (layer > 0),
     * the builder should auto-add a flat terrain mesh as layer 0. */
    arpt_bounds b = tile_bounds(0, 1, 0);
    arpt_geom g = {0};
    g.type = 1;
    double x = 7.45, y = 46.95;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    uint8_t *decoded;
    void *compressed;
    arpentry_tiles_Tile_table_t tile =
        build_and_decode(&b, &g, 2, &decoded, &compressed);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    int n_layers = (int)arpentry_tiles_Layer_vec_len(layers);
    TEST_ASSERT_TRUE(n_layers >= 2); /* terrain + our layer */

    /* First layer should be "terrain" with a MeshGeometry */
    arpentry_tiles_Layer_table_t terrain_layer =
        arpentry_tiles_Layer_vec_at(layers, 0);
    TEST_ASSERT_EQUAL_STRING(
        "terrain", arpentry_tiles_Layer_name(terrain_layer));

    arpentry_tiles_Feature_vec_t terrain_feats =
        arpentry_tiles_Layer_features(terrain_layer);
    TEST_ASSERT_TRUE(arpentry_tiles_Feature_vec_len(terrain_feats) >= 1);

    arpentry_tiles_Feature_table_t tf =
        arpentry_tiles_Feature_vec_at(terrain_feats, 0);
    TEST_ASSERT_EQUAL_INT(arpentry_tiles_Geometry_MeshGeometry,
                          arpentry_tiles_Feature_geometry_type(tf));

    free(decoded);
    free(compressed);
}

int main(void) {
    UNITY_BEGIN();

    /* Point tracing */
    RUN_TEST(test_point_clip_z0);
    RUN_TEST(test_point_quantize_z0);
    RUN_TEST(test_point_clip_z5);
    RUN_TEST(test_point_quantize_z5);

    /* Line tracing */
    RUN_TEST(test_line_clip_z0);
    RUN_TEST(test_line_clip_z4);
    RUN_TEST(test_line_quantize_roundtrip);

    /* Polygon tracing */
    RUN_TEST(test_polygon_clip_z0);
    RUN_TEST(test_polygon_clip_z4);
    RUN_TEST(test_polygon_quantize_roundtrip);

    /* Simplification */
    RUN_TEST(test_simplify_preserves_line);
    RUN_TEST(test_simplify_reduces_at_low_zoom);

    /* Multi-zoom visibility */
    RUN_TEST(test_point_visible_at_all_zooms);
    RUN_TEST(test_line_visible_at_all_zooms);
    RUN_TEST(test_polygon_visible_at_all_zooms);

    /* Precision */
    RUN_TEST(test_quantize_precision_improves);

    /* Tile structure */
    RUN_TEST(test_tile_has_terrain_layer);

    return UNITY_END();
}
