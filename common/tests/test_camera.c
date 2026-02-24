#include "unity.h"
#include "camera.h"
#include "globe.h"
#include <math.h>

#define DEG2RAD(d) ((d) * M_PI / 180.0)

void setUp(void) {}
void tearDown(void) {}

/* ── Defaults ──────────────────────────────────────────────────────────── */

void test_camera_defaults(void) {
    arpt_camera *cam = arpt_camera_create();
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_lon(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_lat(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 10000000.0, arpt_camera_altitude(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_tilt(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_bearing(cam));
    arpt_camera_free(cam);
}

/* ── Projection ────────────────────────────────────────────────────────── */

void test_projection_near_far(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 10000.0);
    arpt_camera_set_viewport(cam, 1920, 1080);
    arpt_mat4 p = arpt_camera_projection(cam);

    /* Near = max(1, alt*0.01) = 100, Far = alt*10 = 100000 */
    /* Check that near maps to z=0 in NDC */
    float z_near = p.m[10] * (-100.0f) + p.m[14];
    float w_near = p.m[11] * (-100.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, z_near / w_near);

    arpt_camera_free(cam);
}

/* ── Tile model matrix ─────────────────────────────────────────────────── */

void test_tile_model_at_interest(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(7.4), DEG2RAD(46.9), 5000.0);

    /* Tile centered at the interest point itself */
    arpt_mat4 m = arpt_camera_tile_model(cam, DEG2RAD(7.4), DEG2RAD(46.9), 0.0);

    /* Translation should be (0, 0, -altitude) since delta = 0 */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, m.m[12]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, m.m[13]);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, -5000.0f, m.m[14]);

    arpt_camera_free(cam);
}

void test_tile_model_nearby_tile(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 5000.0);

    /* Tile slightly east of interest */
    arpt_mat4 m = arpt_camera_tile_model(cam, DEG2RAD(0.01), 0.0, 0.0);

    /* x should be positive (tile is east → +X), z ≈ -altitude */
    TEST_ASSERT_TRUE(m.m[12] > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(100.0f, -5000.0f, m.m[14]);

    arpt_camera_free(cam);
}

/* ── Globe rotation correctness ────────────────────────────────────────── */

void test_screen_center_ray_along_neg_z(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(10.0), DEG2RAD(45.0), 50000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    arpt_dvec3 origin, dir;
    bool ok = arpt_camera_screen_to_ray(cam, 400.0, 300.0, &origin, &dir);
    TEST_ASSERT_TRUE(ok);

    /* The center ray should hit near the interest point.
       Intersect with ellipsoid and check geodetic coords. */
    double t;
    bool hit = arpt_ray_ellipsoid(origin, dir, &t);
    TEST_ASSERT_TRUE(hit);

    arpt_dvec3 hitpt = arpt_dvec3_add(origin, arpt_dvec3_scale(dir, t));
    double lon, lat, alt;
    arpt_ecef_to_geodetic(hitpt, &lon, &lat, &alt);

    /* Should be close to the camera's interest point */
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(1.0), DEG2RAD(10.0), lon);
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(1.0), DEG2RAD(45.0), lat);

    arpt_camera_free(cam);
}

/* ── Zoom level ────────────────────────────────────────────────────────── */

void test_zoom_level_high_altitude(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 20000000.0);  /* 20,000 km */
    arpt_camera_set_viewport(cam, 800, 600);
    int level = arpt_camera_zoom_level(cam, 50000.0, 0, 16);
    /* At very high altitude, should be level 0 or 1 */
    TEST_ASSERT_TRUE(level <= 2);
    arpt_camera_free(cam);
}

void test_zoom_level_low_altitude(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 500.0);
    arpt_camera_set_viewport(cam, 1920, 1080);
    int level = arpt_camera_zoom_level(cam, 50000.0, 0, 16);
    /* At 500m, should be a high zoom level */
    TEST_ASSERT_TRUE(level >= 10);
    arpt_camera_free(cam);
}

void test_zoom_level_clamped(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 100.0);
    arpt_camera_set_viewport(cam, 1920, 1080);
    int level = arpt_camera_zoom_level(cam, 50000.0, 0, 16);
    TEST_ASSERT_TRUE(level <= 16);
    arpt_camera_free(cam);
}

/* ── Pan / Zoom / Tilt ─────────────────────────────────────────────────── */

void test_pan_changes_position(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 5000.0);
    double orig_lon = arpt_camera_lon(cam);
    arpt_camera_pan(cam, 100.0, 0.0);
    TEST_ASSERT_TRUE(arpt_camera_lon(cam) != orig_lon);
    arpt_camera_free(cam);
}

void test_zoom_changes_altitude(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 5000.0);
    arpt_camera_zoom(cam, 0.9);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 4500.0, arpt_camera_altitude(cam));
    arpt_camera_free(cam);
}

void test_tilt_bearing(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_tilt_bearing(cam, DEG2RAD(30.0), DEG2RAD(45.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, DEG2RAD(30.0), arpt_camera_tilt(cam));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, DEG2RAD(45.0), arpt_camera_bearing(cam));

    /* Tilt should clamp at 60° */
    arpt_camera_tilt_bearing(cam, DEG2RAD(40.0), 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, DEG2RAD(60.0), arpt_camera_tilt(cam));
    arpt_camera_free(cam);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_camera_defaults);
    RUN_TEST(test_projection_near_far);
    RUN_TEST(test_tile_model_at_interest);
    RUN_TEST(test_tile_model_nearby_tile);
    RUN_TEST(test_screen_center_ray_along_neg_z);
    RUN_TEST(test_zoom_level_high_altitude);
    RUN_TEST(test_zoom_level_low_altitude);
    RUN_TEST(test_zoom_level_clamped);
    RUN_TEST(test_pan_changes_position);
    RUN_TEST(test_zoom_changes_altitude);
    RUN_TEST(test_tilt_bearing);
    return UNITY_END();
}
