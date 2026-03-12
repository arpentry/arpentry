/* Integration tests: Natural Earth 110m GeoParquet fixtures through the
 * tiler pipeline.  Verifies reader, pipeline, and geographic correctness. */

#include "unity.h"
#include "archive.h"
#include "overture.h"
#include "pipeline.h"
#include "tile.h"
#include "tile_reader.h"

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
        TEST_ASSERT_TRUE_MESSAGE(feat.geometry.n_coords > 0,
            "Feature has zero coordinates");

        uint32_t type_bit = 1u << feat.geometry.type;
        if (!(type_bit & allowed_types)) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                "Unexpected geometry type %u in %s", feat.geometry.type,
                filename);
            TEST_FAIL_MESSAGE(msg);
        }

        /* Natural Earth 110m data can exceed standard ranges near the
         * antimeridian and at the poles (e.g., Russia wraps past 180,
         * Antarctica extends to ~-90.5 in some datasets). Use generous
         * ranges that still catch truly broken coordinates. */
        for (uint32_t i = 0; i < feat.geometry.n_coords; i++) {
            TEST_ASSERT_TRUE_MESSAGE(
                feat.geometry.x[i] >= -360.0 && feat.geometry.x[i] <= 360.0,
                "x coordinate out of lon range");
            TEST_ASSERT_TRUE_MESSAGE(
                feat.geometry.y[i] >= -91.0 && feat.geometry.y[i] <= 91.0,
                "y coordinate out of lat range");
        }

        arpt_geom_free(&feat.geometry);
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

    /* Cleanup */
    RUN_TEST(test_cleanup);

    return UNITY_END();
}
