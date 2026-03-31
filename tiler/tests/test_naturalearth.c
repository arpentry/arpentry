/* Integration tests: Natural Earth 110m GeoParquet fixtures through the
 * tiler pipeline.  Verifies reader, pipeline, and geographic correctness. */

#include "unity.h"
#include "archive.h"
#include "overture.h"
#include "pipeline.h"
#include "tile.h"
#include "tile_reader.h"
#include "wkb.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures/naturalearth"
#endif

static const char *ARCHIVE_PATH = "/tmp/test_naturalearth.arpa";

void setUp(void) {}
void tearDown(void) {}

/* ── WKB geometry type constants ─────────────────────────────────────── */

#define WKB_POINT          1
#define WKB_LINESTRING     2
#define WKB_POLYGON        3
#define WKB_MULTIPOINT     4
#define WKB_MULTILINESTRING 5
#define WKB_MULTIPOLYGON   6

/* ── Tile constants ──────────────────────────────────────────────────── */

#define TILE_EXTENT 32768
#define TILE_BUFFER 16384

/* ── Helpers ─────────────────────────────────────────────────────────── */

static void read_all_features(const char *filename,
                              uint32_t allowed_types,
                              int min_expected)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", FIXTURE_DIR, filename);

    arpt_overture *ov = arpt_overture_open(path);
    if (!ov) {
        char msg[600];
        snprintf(msg, sizeof(msg), "Failed to open %s", path);
        TEST_FAIL_MESSAGE(msg);
        return;
    }

    int count = 0;
    arpt_overture_feature feat;
    while (arpt_overture_next(ov, &feat)) {
        arpt_geom geom = {0};
        TEST_ASSERT_TRUE_MESSAGE(
            arpt_wkb_parse(feat.wkb, feat.wkb_len, &geom),
            "WKB parse failed");

        TEST_ASSERT_TRUE_MESSAGE(geom.n_coords > 0,
            "Feature has zero coordinates");

        uint32_t type_bit = 1u << geom.type;
        if (!(type_bit & allowed_types)) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                "Unexpected geometry type %u in %s", geom.type,
                filename);
            TEST_FAIL_MESSAGE(msg);
        }

        /* Natural Earth 110m data can exceed standard ranges near the
         * antimeridian and at the poles (e.g., Russia wraps past 180,
         * Antarctica extends to ~-90.5 in some datasets). Use generous
         * ranges that still catch truly broken coordinates. */
        for (uint32_t i = 0; i < geom.n_coords; i++) {
            TEST_ASSERT_TRUE_MESSAGE(
                geom.x[i] >= -360.0 && geom.x[i] <= 360.0,
                "x coordinate out of lon range");
            TEST_ASSERT_TRUE_MESSAGE(
                geom.y[i] >= -91.0 && geom.y[i] <= 91.0,
                "y coordinate out of lat range");
        }

        arpt_geom_free(&geom);
        count++;
    }

    char msg[128];
    snprintf(msg, sizeof(msg), "%s: expected >= %d features, got %d",
             filename, min_expected, count);
    TEST_ASSERT_TRUE_MESSAGE(count >= min_expected, msg);
    arpt_overture_close(ov);
}

/* Dequantize tile coords to WGS84. */
static double dequant_x(double min_x, double max_x, uint16_t qx) {
    return min_x +
           ((double)qx - TILE_BUFFER) / TILE_EXTENT * (max_x - min_x);
}

static double dequant_y(double min_y, double max_y, uint16_t qy) {
    return min_y +
           ((double)qy - TILE_BUFFER) / TILE_EXTENT * (max_y - min_y);
}

/* ── A. Reader tests ─────────────────────────────────────────────────── */

static void test_read_land(void) {
    read_all_features("land.parquet",
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_read_coastline(void) {
    read_all_features("coastline.parquet",
        (1u << WKB_LINESTRING) | (1u << WKB_MULTILINESTRING),
        100);
}

static void test_read_boundary(void) {
    read_all_features("boundary.parquet",
        (1u << WKB_LINESTRING) | (1u << WKB_MULTILINESTRING),
        100);
}

static void test_read_places(void) {
    read_all_features("places.parquet",
        (1u << WKB_POINT),
        200);
}

/* ── B. Pipeline end-to-end ──────────────────────────────────────────── */

/* Run the pipeline once and cache the result for subsequent tests.
 * Returns the archive reader (NULL on failure). */
static arpt_archive_reader *cached_reader = NULL;
static bool pipeline_ran = false;

static arpt_archive_reader *get_archive(void) {
    if (!pipeline_ran) {
        pipeline_ran = true;

        char land_path[512], coast_path[512], bound_path[512], place_path[512];
        snprintf(land_path, sizeof(land_path), "%s/land.parquet", FIXTURE_DIR);
        snprintf(coast_path, sizeof(coast_path), "%s/coastline.parquet",
                 FIXTURE_DIR);
        snprintf(bound_path, sizeof(bound_path), "%s/boundary.parquet",
                 FIXTURE_DIR);
        snprintf(place_path, sizeof(place_path), "%s/places.parquet",
                 FIXTURE_DIR);

        arpt_pipeline_input inputs[] = {
            { .path = land_path,  .layer = 1 },  /* surface */
            { .path = coast_path, .layer = 1 },  /* surface */
            { .path = bound_path, .layer = 1 },  /* surface */
            { .path = place_path, .layer = 5 },  /* poi */
        };

        arpt_pipeline_config cfg = {
            .output = ARCHIVE_PATH,
            .tmp_dir = "/tmp",
            .mem_budget = 4 * 1024 * 1024,
            .bbox = {-180.0, -85.0, 180.0, 85.0},
            .min_zoom = 0,
            .max_zoom = 4,
            .synthetic = false,
            .inputs = inputs,
            .n_inputs = 4,
        };

        if (arpt_pipeline_run(&cfg)) {
            cached_reader = arpt_archive_reader_open(ARCHIVE_PATH);
        }
    }
    return cached_reader;
}

static void test_pipeline_produces_archive(void) {
    arpt_archive_reader *r = get_archive();
    TEST_ASSERT_NOT_NULL_MESSAGE(r, "Pipeline failed to produce archive");
}

static void test_pipeline_has_z0_tile(void) {
    arpt_archive_reader *r = get_archive();
    TEST_ASSERT_NOT_NULL(r);

    size_t tile_size;
    const void *tile = arpt_archive_reader_get_tile(r, 0, 0, 0, &tile_size);
    TEST_ASSERT_NOT_NULL_MESSAGE(tile, "Tile (0,0,0) not found in archive");
    TEST_ASSERT_TRUE(tile_size > 0);
}

static void test_pipeline_tile_count(void) {
    arpt_archive_reader *r = get_archive();
    TEST_ASSERT_NOT_NULL(r);

    uint64_t count = arpt_archive_reader_tile_count(r);
    /* With world data at z0–z4, expect many tiles */
    TEST_ASSERT_TRUE_MESSAGE(count >= 5,
        "Expected at least 5 tiles across z0–z4");
}

/* ── C. Geographic content assertions ────────────────────────────────── */

/* Decode tile (z,x,y) from the cached archive.
 * Sets *decoded (caller frees) and returns the Tile table, or NULL. */
static arpentry_tiles_Tile_table_t decode_tile(uint8_t z, uint32_t x,
                                                uint32_t y,
                                                uint8_t **decoded_out) {
    arpt_archive_reader *r = get_archive();
    if (!r) return NULL;

    size_t tile_size;
    const void *tile_data = arpt_archive_reader_get_tile(r, z, x, y,
                                                          &tile_size);
    if (!tile_data) return NULL;

    uint8_t *decoded;
    size_t decoded_size;
    if (!arpt_decode(tile_data, tile_size, &decoded, &decoded_size))
        return NULL;

    *decoded_out = decoded;
    return arpentry_tiles_Tile_as_root(decoded);
}

/* Find a layer by name in a decoded tile. */
static arpentry_tiles_Layer_table_t find_layer(
    arpentry_tiles_Tile_table_t tile, const char *name)
{
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    if (!layers) return NULL;
    int n = (int)arpentry_tiles_Layer_vec_len(layers);
    for (int i = 0; i < n; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        const char *lname = arpentry_tiles_Layer_name(layer);
        if (lname && strcmp(lname, name) == 0)
            return layer;
    }
    return NULL;
}

static void test_z0_has_surface_layer(void) {
    uint8_t *decoded;
    arpentry_tiles_Tile_table_t tile = decode_tile(0, 0, 0, &decoded);
    TEST_ASSERT_NOT_NULL_MESSAGE(tile, "Could not decode tile (0,0,0)");

    arpentry_tiles_Layer_table_t surface = find_layer(tile, "surface");
    TEST_ASSERT_NOT_NULL_MESSAGE(surface,
        "Tile (0,0,0) missing 'surface' layer");

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(surface);
    TEST_ASSERT_NOT_NULL(features);
    TEST_ASSERT_TRUE_MESSAGE(
        arpentry_tiles_Feature_vec_len(features) > 0,
        "Surface layer has no features");

    free(decoded);
}

static void test_z0_has_poi_layer(void) {
    uint8_t *decoded;
    arpentry_tiles_Tile_table_t tile = decode_tile(0, 0, 0, &decoded);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_table_t poi = find_layer(tile, "poi");
    TEST_ASSERT_NOT_NULL_MESSAGE(poi, "Tile (0,0,0) missing 'poi' layer");

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(poi);
    TEST_ASSERT_NOT_NULL(features);
    int n = (int)arpentry_tiles_Feature_vec_len(features);
    TEST_ASSERT_TRUE_MESSAGE(n > 0, "POI layer has no features");

    /* All POI features should be PointGeometry */
    for (int i = 0; i < n; i++) {
        arpentry_tiles_Feature_table_t f =
            arpentry_tiles_Feature_vec_at(features, i);
        TEST_ASSERT_EQUAL_INT(arpentry_tiles_Geometry_PointGeometry,
                              arpentry_tiles_Feature_geometry_type(f));
    }

    free(decoded);
}

static void test_z0_has_polygon_geometry(void) {
    uint8_t *decoded;
    arpentry_tiles_Tile_table_t tile = decode_tile(0, 0, 0, &decoded);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_table_t surface = find_layer(tile, "surface");
    TEST_ASSERT_NOT_NULL(surface);

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(surface);
    int n = (int)arpentry_tiles_Feature_vec_len(features);

    bool found_polygon = false;
    for (int i = 0; i < n; i++) {
        arpentry_tiles_Feature_table_t f =
            arpentry_tiles_Feature_vec_at(features, i);
        if (arpentry_tiles_Feature_geometry_type(f) ==
            arpentry_tiles_Geometry_PolygonGeometry) {
            found_polygon = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_polygon,
        "Surface layer has no PolygonGeometry (expected from land)");

    free(decoded);
}

static void test_z0_has_line_geometry(void) {
    uint8_t *decoded;
    arpentry_tiles_Tile_table_t tile = decode_tile(0, 0, 0, &decoded);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_table_t surface = find_layer(tile, "surface");
    TEST_ASSERT_NOT_NULL(surface);

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(surface);
    int n = (int)arpentry_tiles_Feature_vec_len(features);

    bool found_line = false;
    for (int i = 0; i < n; i++) {
        arpentry_tiles_Feature_table_t f =
            arpentry_tiles_Feature_vec_at(features, i);
        if (arpentry_tiles_Feature_geometry_type(f) ==
            arpentry_tiles_Geometry_LineGeometry) {
            found_line = true;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_line,
        "Surface layer has no LineGeometry (expected from coastline/boundary)");

    free(decoded);
}

static void test_z0_covers_all_quadrants(void) {
    uint8_t *decoded;
    arpentry_tiles_Tile_table_t tile = decode_tile(0, 0, 0, &decoded);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_table_t surface = find_layer(tile, "surface");
    TEST_ASSERT_NOT_NULL(surface);

    /* z=0 tile bounds: lon [-180, 180], lat [~-85.05, ~85.05] */
    double min_x = -180.0, max_x = 180.0;
    double n_val = (double)(1 << 0);
    double min_y = atan(sinh(M_PI * (1.0 - 2.0 * 1.0 / n_val))) * 180.0 / M_PI;
    double max_y = atan(sinh(M_PI * (1.0 - 2.0 * 0.0 / n_val))) * 180.0 / M_PI;

    /* Track which quadrants we've seen coords in */
    bool ne = false, nw = false, se = false, sw = false;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(surface);
    int n = (int)arpentry_tiles_Feature_vec_len(features);

    for (int i = 0; i < n; i++) {
        arpentry_tiles_Feature_table_t f =
            arpentry_tiles_Feature_vec_at(features, i);
        uint8_t gtype = arpentry_tiles_Feature_geometry_type(f);

        flatbuffers_uint16_vec_t xs = NULL;
        flatbuffers_uint16_vec_t ys = NULL;
        int ncoords = 0;

        if (gtype == arpentry_tiles_Geometry_PolygonGeometry) {
            arpentry_tiles_PolygonGeometry_table_t pg =
                (arpentry_tiles_PolygonGeometry_table_t)
                    arpentry_tiles_Feature_geometry(f);
            xs = arpentry_tiles_PolygonGeometry_x(pg);
            ys = arpentry_tiles_PolygonGeometry_y(pg);
        } else if (gtype == arpentry_tiles_Geometry_LineGeometry) {
            arpentry_tiles_LineGeometry_table_t lg =
                (arpentry_tiles_LineGeometry_table_t)
                    arpentry_tiles_Feature_geometry(f);
            xs = arpentry_tiles_LineGeometry_x(lg);
            ys = arpentry_tiles_LineGeometry_y(lg);
        } else if (gtype == arpentry_tiles_Geometry_PointGeometry) {
            arpentry_tiles_PointGeometry_table_t pg =
                (arpentry_tiles_PointGeometry_table_t)
                    arpentry_tiles_Feature_geometry(f);
            xs = arpentry_tiles_PointGeometry_x(pg);
            ys = arpentry_tiles_PointGeometry_y(pg);
        } else {
            continue;
        }

        if (!xs || !ys) continue;
        ncoords = (int)flatbuffers_uint16_vec_len(xs);

        for (int j = 0; j < ncoords; j++) {
            double lon = dequant_x(min_x, max_x,
                                   flatbuffers_uint16_vec_at(xs, j));
            double lat = dequant_y(min_y, max_y,
                                   flatbuffers_uint16_vec_at(ys, j));

            if (lon > 0 && lat > 0) ne = true;
            if (lon < 0 && lat > 0) nw = true;
            if (lon > 0 && lat < 0) se = true;
            if (lon < 0 && lat < 0) sw = true;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(ne, "No coordinates in NE quadrant");
    TEST_ASSERT_TRUE_MESSAGE(nw, "No coordinates in NW quadrant");
    TEST_ASSERT_TRUE_MESSAGE(se, "No coordinates in SE quadrant");
    TEST_ASSERT_TRUE_MESSAGE(sw, "No coordinates in SW quadrant");

    free(decoded);
}

static void test_z0_terrain_mesh(void) {
    uint8_t *decoded;
    arpentry_tiles_Tile_table_t tile = decode_tile(0, 0, 0, &decoded);
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_table_t terrain = find_layer(tile, "terrain");
    TEST_ASSERT_NOT_NULL_MESSAGE(terrain,
        "Tile (0,0,0) missing 'terrain' layer");

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(terrain);
    TEST_ASSERT_NOT_NULL(features);
    TEST_ASSERT_TRUE(arpentry_tiles_Feature_vec_len(features) >= 1);

    arpentry_tiles_Feature_table_t tf =
        arpentry_tiles_Feature_vec_at(features, 0);
    TEST_ASSERT_EQUAL_INT(arpentry_tiles_Geometry_MeshGeometry,
                          arpentry_tiles_Feature_geometry_type(tf));

    free(decoded);
}

/* ── D. Clip diagnostics ─────────────────────────────────────────────── */

#include "clip.h"

typedef struct {
    int z, x, y;
    double *cx, *cy;
    uint32_t *coffsets;
    uint32_t n_coords, n_offsets;
} diag_result;

typedef struct {
    diag_result *results;
    int count, cap;
} diag_collector;

static void diag_cb(int z, int x, int y,
                    const arpt_geom *clipped, void *ctx) {
    diag_collector *dc = (diag_collector *)ctx;
    if (dc->count == dc->cap) {
        int nc = dc->cap ? dc->cap * 2 : 64;
        dc->results = realloc(dc->results, (size_t)nc * sizeof(diag_result));
        dc->cap = nc;
    }
    diag_result *r = &dc->results[dc->count++];
    r->z = z; r->x = x; r->y = y;
    r->n_coords = clipped->n_coords;
    r->cx = malloc(clipped->n_coords * sizeof(double));
    r->cy = malloc(clipped->n_coords * sizeof(double));
    memcpy(r->cx, clipped->x, clipped->n_coords * sizeof(double));
    memcpy(r->cy, clipped->y, clipped->n_coords * sizeof(double));
    if (clipped->offsets && clipped->n_offsets > 0) {
        r->n_offsets = clipped->n_offsets;
        r->coffsets = malloc(clipped->n_offsets * sizeof(uint32_t));
        memcpy(r->coffsets, clipped->offsets,
               clipped->n_offsets * sizeof(uint32_t));
    } else {
        r->n_offsets = 0;
        r->coffsets = NULL;
    }
}

/* Compute signed area of a closed ring */
static double diag_ring_area(const double *x, const double *y,
                              uint32_t start, uint32_t end) {
    double area = 0.0;
    for (uint32_t i = start; i < end - 1; i++) {
        area += x[i] * y[i + 1] - x[i + 1] * y[i];
    }
    return area * 0.5;
}

/* Test: clip the NE land polygons to Gulf of Mexico tiles and check
 * that the Gulf is NOT filled with land. */
static void test_gulf_clip_diagnostic(void) {
    char path[512];
    snprintf(path, sizeof(path), "%s/land.parquet", FIXTURE_DIR);

    arpt_overture *ov = arpt_overture_open(path);
    TEST_ASSERT_NOT_NULL(ov);

    /* Read all land features */
    arpt_overture_feature feat;
    diag_collector dc = {NULL, 0, 0};

    while (arpt_overture_next(ov, &feat)) {
        arpt_geom g = {0};
        if (!arpt_wkb_parse(feat.wkb, feat.wkb_len, &g)) continue;

        /* Only clip features that overlap the Gulf area
         * Gulf bbox: lon [-100, -80], lat [18, 31] */
        double gmin_x = g.x[0], gmax_x = g.x[0];
        double gmin_y = g.y[0], gmax_y = g.y[0];
        for (uint32_t i = 1; i < g.n_coords; i++) {
            if (g.x[i] < gmin_x) gmin_x = g.x[i];
            if (g.x[i] > gmax_x) gmax_x = g.x[i];
            if (g.y[i] < gmin_y) gmin_y = g.y[i];
            if (g.y[i] > gmax_y) gmax_y = g.y[i];
        }

        /* Skip features that don't overlap the Gulf area */
        if (gmax_x < -100.0 || gmin_x > -80.0 ||
            gmax_y < 18.0 || gmin_y > 31.0) {
            arpt_geom_free(&g);
            continue;
        }

        fprintf(stderr, "  Clipping feature type=%u n_coords=%u "
                "n_offsets=%u bbox=[%.1f,%.1f,%.1f,%.1f]\n",
                g.type, g.n_coords, g.n_offsets,
                gmin_x, gmin_y, gmax_x, gmax_y);

        /* Clip at zoom 3 and 4 */
        arpt_assign_tiles(&g, &g, 3, diag_cb, &dc);
        arpt_assign_tiles(&g, &g, 4, diag_cb, &dc);
        arpt_geom_free(&g);
    }
    arpt_overture_close(ov);

    fprintf(stderr, "  Total clipped results: %d\n", dc.count);

    /* At z=3, the Gulf center (~lon -90, lat 25) falls in:
     * col = floor((-90 + 180) / 22.5) = floor(90/22.5) = 4
     * row = floor((25 + 90) / 22.5) = floor(115/22.5) = 5
     * So tile (3, 4, 5) covers lon [-90, -67.5], lat [22.5, 45] */
    for (int i = 0; i < dc.count; i++) {
        diag_result *r = &dc.results[i];
        /* Print results for tiles in the Gulf area */
        if (((r->z == 3 && r->x >= 3 && r->x <= 5 &&
              r->y >= 4 && r->y <= 5) ||
             (r->z == 4 && r->x >= 6 && r->x <= 10 &&
              r->y >= 9 && r->y <= 11))) {
            fprintf(stderr, "  Tile (%d,%d,%d): %u coords, %u offsets\n",
                    r->z, r->x, r->y, r->n_coords, r->n_offsets);

            /* Print ring areas */
            if (r->n_offsets >= 2) {
                for (uint32_t ri = 0; ri + 1 < r->n_offsets; ri++) {
                    uint32_t start = r->coffsets[ri];
                    uint32_t end = r->coffsets[ri + 1];
                    double area = diag_ring_area(r->cx, r->cy, start, end);
                    fprintf(stderr, "    ring %u: [%u..%u] %u verts, "
                            "area=%.4f\n",
                            ri, start, end, end - start, area);
                    /* Print vertices — all for small rings, first/last for big */
                    uint32_t nv = end - start;
                    if (nv <= 30) {
                        for (uint32_t j = start; j < end; j++) {
                            fprintf(stderr, "      [%u] (%.4f, %.4f)\n",
                                    j, r->cx[j], r->cy[j]);
                        }
                    } else {
                        for (uint32_t j = start; j < start + 5; j++) {
                            fprintf(stderr, "      [%u] (%.4f, %.4f)\n",
                                    j, r->cx[j], r->cy[j]);
                        }
                        fprintf(stderr, "      ... (%u more) ...\n", nv - 10);
                        for (uint32_t j = end - 5; j < end; j++) {
                            fprintf(stderr, "      [%u] (%.4f, %.4f)\n",
                                    j, r->cx[j], r->cy[j]);
                        }
                    }
                }
            }
        }
    }

    /* Cleanup */
    for (int i = 0; i < dc.count; i++) {
        free(dc.results[i].cx);
        free(dc.results[i].cy);
        free(dc.results[i].coffsets);
    }
    free(dc.results);

    TEST_PASS();
}

/* Test: clip European land polygons at z=4 and check for artifacts.
 * The camera view at lon=15, lat=52 shows tiles 4/15-19/11-14.
 * A blue wedge artifact appears at z=4 but not z=3. */
static void test_europe_clip_diagnostic_z4(void) {
    char path[512];
    snprintf(path, sizeof(path), "%s/land.parquet", FIXTURE_DIR);

    arpt_overture *ov = arpt_overture_open(path);
    TEST_ASSERT_NOT_NULL(ov);

    arpt_overture_feature feat;
    diag_collector dc = {NULL, 0, 0};

    while (arpt_overture_next(ov, &feat)) {
        arpt_geom g = {0};
        if (!arpt_wkb_parse(feat.wkb, feat.wkb_len, &g)) continue;

        /* Only clip features that overlap Europe: lon [-15, 50], lat [30, 75] */
        double gmin_x = g.x[0], gmax_x = g.x[0];
        double gmin_y = g.y[0], gmax_y = g.y[0];
        for (uint32_t i = 1; i < g.n_coords; i++) {
            if (g.x[i] < gmin_x) gmin_x = g.x[i];
            if (g.x[i] > gmax_x) gmax_x = g.x[i];
            if (g.y[i] < gmin_y) gmin_y = g.y[i];
            if (g.y[i] > gmax_y) gmax_y = g.y[i];
        }

        if (gmax_x < -15.0 || gmin_x > 50.0 ||
            gmax_y < 30.0 || gmin_y > 75.0) {
            arpt_geom_free(&g);
            continue;
        }

        fprintf(stderr, "  EUR feature type=%u n_coords=%u "
                "n_offsets=%u bbox=[%.1f,%.1f,%.1f,%.1f]\n",
                g.type, g.n_coords, g.n_offsets,
                gmin_x, gmin_y, gmax_x, gmax_y);

        arpt_assign_tiles(&g, &g, 4, diag_cb, &dc);
        arpt_geom_free(&g);
    }
    arpt_overture_close(ov);

    fprintf(stderr, "  EUR z=4 total clipped results: %d\n", dc.count);

    /* Check tiles visible in the artifact screenshot:
     * Tiles 4/15-19/11-14 (lon [-11.25, 45], lat [33.75, 78.75]) */
    int issue_count = 0;
    for (int i = 0; i < dc.count; i++) {
        diag_result *r = &dc.results[i];
        if (r->z != 4 || r->x < 15 || r->x > 19 || r->y < 11 || r->y > 14)
            continue;

        /* Check every ring for validity */
        if (r->n_offsets >= 2) {
            for (uint32_t ri = 0; ri + 1 < r->n_offsets; ri++) {
                uint32_t start = r->coffsets[ri];
                uint32_t end = r->coffsets[ri + 1];
                uint32_t nv = end - start;

                /* Check ring closure */
                if (nv >= 4) {
                    if (r->cx[start] != r->cx[end - 1] ||
                        r->cy[start] != r->cy[end - 1]) {
                        fprintf(stderr, "  *** UNCLOSED ring: tile(%d,%d,%d) "
                                "ring %u: first(%.6f,%.6f) last(%.6f,%.6f)\n",
                                r->z, r->x, r->y, ri,
                                r->cx[start], r->cy[start],
                                r->cx[end-1], r->cy[end-1]);
                        issue_count++;
                    }

                    /* Check for consecutive duplicate vertices */
                    for (uint32_t j = start; j + 1 < end - 1; j++) {
                        if (r->cx[j] == r->cx[j+1] &&
                            r->cy[j] == r->cy[j+1]) {
                            fprintf(stderr, "  *** DUPLICATE: tile(%d,%d,%d) "
                                    "ring %u vert %u: (%.6f,%.6f)\n",
                                    r->z, r->x, r->y, ri, j,
                                    r->cx[j], r->cy[j]);
                            issue_count++;
                        }
                    }

                    /* Check for self-overlap: look for pairs of
                     * boundary edges on the same line with overlapping
                     * parameter ranges (the original bug). */
                    for (uint32_t j = start; j + 1 < end; j++) {
                        for (uint32_t k = j + 2; k + 1 < end; k++) {
                            /* Check if edges j and k are both vertical
                             * at the same x (left/right boundary overlap) */
                            if (r->cx[j] == r->cx[j+1] &&
                                r->cx[k] == r->cx[k+1] &&
                                r->cx[j] == r->cx[k]) {
                                double j_lo = r->cy[j] < r->cy[j+1] ?
                                    r->cy[j] : r->cy[j+1];
                                double j_hi = r->cy[j] > r->cy[j+1] ?
                                    r->cy[j] : r->cy[j+1];
                                double k_lo = r->cy[k] < r->cy[k+1] ?
                                    r->cy[k] : r->cy[k+1];
                                double k_hi = r->cy[k] > r->cy[k+1] ?
                                    r->cy[k] : r->cy[k+1];
                                if (j_lo < k_hi && k_lo < j_hi) {
                                    fprintf(stderr,
                                        "  *** OVERLAP: tile(%d,%d,%d) "
                                        "ring %u edges [%u] and [%u] "
                                        "overlap at x=%.4f y=[%.4f,%.4f]\n",
                                        r->z, r->x, r->y, ri, j, k,
                                        r->cx[j], fmax(j_lo, k_lo),
                                        fmin(j_hi, k_hi));
                                    issue_count++;
                                }
                            }
                            /* Check horizontal overlap (top/bottom) */
                            if (r->cy[j] == r->cy[j+1] &&
                                r->cy[k] == r->cy[k+1] &&
                                r->cy[j] == r->cy[k]) {
                                double j_lo = r->cx[j] < r->cx[j+1] ?
                                    r->cx[j] : r->cx[j+1];
                                double j_hi = r->cx[j] > r->cx[j+1] ?
                                    r->cx[j] : r->cx[j+1];
                                double k_lo = r->cx[k] < r->cx[k+1] ?
                                    r->cx[k] : r->cx[k+1];
                                double k_hi = r->cx[k] > r->cx[k+1] ?
                                    r->cx[k] : r->cx[k+1];
                                if (j_lo < k_hi && k_lo < j_hi) {
                                    fprintf(stderr,
                                        "  *** OVERLAP: tile(%d,%d,%d) "
                                        "ring %u edges [%u] and [%u] "
                                        "overlap at y=%.4f x=[%.4f,%.4f]\n",
                                        r->z, r->x, r->y, ri, j, k,
                                        r->cy[j], fmax(j_lo, k_lo),
                                        fmin(j_hi, k_hi));
                                    issue_count++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (issue_count > 0) {
        fprintf(stderr, "  EUR z=4 found %d potential issues\n", issue_count);
    } else {
        fprintf(stderr, "  EUR z=4 all rings look valid\n");
    }

    /* Cleanup */
    for (int i = 0; i < dc.count; i++) {
        free(dc.results[i].cx);
        free(dc.results[i].cy);
        free(dc.results[i].coffsets);
    }
    free(dc.results);

    TEST_ASSERT_EQUAL_INT_MESSAGE(0, issue_count,
        "Found clip artifacts in European z=4 tiles");
}

/* ── Cleanup ─────────────────────────────────────────────────────────── */

static void test_cleanup(void) {
    /* Close the cached reader and remove the archive file */
    if (cached_reader) {
        arpt_archive_reader_close(cached_reader);
        cached_reader = NULL;
    }
    remove(ARCHIVE_PATH);
    TEST_PASS();
}

/* ── Main ────────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();

    /* A. Reader tests */
    RUN_TEST(test_read_land);
    RUN_TEST(test_read_coastline);
    RUN_TEST(test_read_boundary);
    RUN_TEST(test_read_places);

    /* B. Pipeline end-to-end */
    RUN_TEST(test_pipeline_produces_archive);
    RUN_TEST(test_pipeline_has_z0_tile);
    RUN_TEST(test_pipeline_tile_count);

    /* C. Geographic content assertions */
    RUN_TEST(test_z0_has_surface_layer);
    RUN_TEST(test_z0_has_poi_layer);
    RUN_TEST(test_z0_has_polygon_geometry);
    RUN_TEST(test_z0_has_line_geometry);
    RUN_TEST(test_z0_covers_all_quadrants);
    RUN_TEST(test_z0_terrain_mesh);

    /* D. Clip diagnostics */
    RUN_TEST(test_gulf_clip_diagnostic);
    RUN_TEST(test_europe_clip_diagnostic_z4);
    /* Cleanup */
    RUN_TEST(test_cleanup);

    return UNITY_END();
}
