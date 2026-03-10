/* Integration tests: read real OvertureMaps GeoParquet samples.
 *
 * Each test opens a small (10-row) sample extracted from the
 * OvertureMaps 2026-02-18.0 release and verifies that the reader
 * can parse every feature without error.
 */

#include "unity.h"
#include "overture.h"
#include <stdio.h>
#include <string.h>

/* Path to fixture directory — set by CMake via -D. */
#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures/overture"
#endif

void setUp(void) {}
void tearDown(void) {}

/* ── Helper ─────────────────────────────────────────────────────────── */

/* Expected WKB geometry types. */
#define WKB_POINT          1
#define WKB_LINESTRING     2
#define WKB_POLYGON        3
#define WKB_MULTIPOINT     4
#define WKB_MULTILINESTRING 5
#define WKB_MULTIPOLYGON   6

/* Read all features from a fixture file.
 * Verifies that every feature has a valid geometry whose type is
 * one of the allowed WKB types in `allowed_types` (bitmask of 1<<type). */
static void read_all_features(const char *filename,
                              uint32_t allowed_types,
                              int expected_count)
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
        /* Geometry must be present and have coordinates */
        TEST_ASSERT_TRUE_MESSAGE(feat.geometry.n_coords > 0,
            "Feature has zero coordinates");

        /* Geometry type must be one of the allowed types */
        uint32_t type_bit = 1u << feat.geometry.type;
        if (!(type_bit & allowed_types)) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                "Unexpected geometry type %u in %s", feat.geometry.type, filename);
            TEST_FAIL_MESSAGE(msg);
        }

        /* Coordinates must be valid lon/lat */
        for (uint32_t i = 0; i < feat.geometry.n_coords; i++) {
            TEST_ASSERT_TRUE_MESSAGE(
                feat.geometry.x[i] >= -180.0 && feat.geometry.x[i] <= 180.0,
                "x coordinate out of lon range");
            TEST_ASSERT_TRUE_MESSAGE(
                feat.geometry.y[i] >= -90.0 && feat.geometry.y[i] <= 90.0,
                "y coordinate out of lat range");
        }

        arpt_geom_free(&feat.geometry);
        count++;
    }

    TEST_ASSERT_EQUAL_INT_MESSAGE(expected_count, count, filename);
    arpt_overture_close(ov);
}

/* ── Addresses ──────────────────────────────────────────────────────── */

static void test_address(void) {
    read_all_features("address.parquet",
        (1u << WKB_POINT),
        100);
}

/* ── Base theme ─────────────────────────────────────────────────────── */

static void test_bathymetry(void) {
    read_all_features("bathymetry.parquet",
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_infrastructure(void) {
    read_all_features("infrastructure.parquet",
        (1u << WKB_POINT) | (1u << WKB_LINESTRING) |
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_land(void) {
    read_all_features("land.parquet",
        (1u << WKB_POINT) | (1u << WKB_LINESTRING) |
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_land_cover(void) {
    read_all_features("land_cover.parquet",
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_land_use(void) {
    read_all_features("land_use.parquet",
        (1u << WKB_POINT) | (1u << WKB_LINESTRING) |
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_water(void) {
    read_all_features("water.parquet",
        (1u << WKB_POINT) | (1u << WKB_LINESTRING) |
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

/* ── Buildings theme ────────────────────────────────────────────────── */

static void test_building(void) {
    read_all_features("building.parquet",
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_building_part(void) {
    read_all_features("building_part.parquet",
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

/* ── Divisions theme ────────────────────────────────────────────────── */

static void test_division(void) {
    read_all_features("division.parquet",
        (1u << WKB_POINT),
        100);
}

static void test_division_area(void) {
    read_all_features("division_area.parquet",
        (1u << WKB_POLYGON) | (1u << WKB_MULTIPOLYGON),
        100);
}

static void test_division_boundary(void) {
    read_all_features("division_boundary.parquet",
        (1u << WKB_LINESTRING) | (1u << WKB_MULTILINESTRING),
        100);
}

/* ── Places theme ───────────────────────────────────────────────────── */

static void test_place(void) {
    read_all_features("place.parquet",
        (1u << WKB_POINT),
        100);
}

/* ── Transportation theme ───────────────────────────────────────────── */

static void test_connector(void) {
    read_all_features("connector.parquet",
        (1u << WKB_POINT),
        100);
}

static void test_segment(void) {
    read_all_features("segment.parquet",
        (1u << WKB_LINESTRING) | (1u << WKB_MULTILINESTRING),
        100);
}

/* ── Main ───────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();

    /* Addresses */
    RUN_TEST(test_address);

    /* Base */
    RUN_TEST(test_bathymetry);
    RUN_TEST(test_infrastructure);
    RUN_TEST(test_land);
    RUN_TEST(test_land_cover);
    RUN_TEST(test_land_use);
    RUN_TEST(test_water);

    /* Buildings */
    RUN_TEST(test_building);
    RUN_TEST(test_building_part);

    /* Divisions */
    RUN_TEST(test_division);
    RUN_TEST(test_division_area);
    RUN_TEST(test_division_boundary);

    /* Places */
    RUN_TEST(test_place);

    /* Transportation */
    RUN_TEST(test_connector);
    RUN_TEST(test_segment);

    return UNITY_END();
}
