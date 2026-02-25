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

/* ── Navigation / inertia ─────────────────────────────────────────────── */

void test_update_no_velocity_no_change(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(10.0), DEG2RAD(45.0), 5000.0);
    arpt_camera_set_viewport(cam, 800, 600);
    double lon_before = arpt_camera_lon(cam);
    double lat_before = arpt_camera_lat(cam);
    double alt_before = arpt_camera_altitude(cam);

    arpt_camera_update(cam, 0.016);

    TEST_ASSERT_DOUBLE_WITHIN(1e-15, lon_before, arpt_camera_lon(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, lat_before, arpt_camera_lat(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, alt_before, arpt_camera_altitude(cam));
    arpt_camera_free(cam);
}

void test_pan_velocity_decays(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 5000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    arpt_camera_set_pan_velocity(cam, 1000.0, 1000.0);
    TEST_ASSERT_TRUE(arpt_camera_is_moving(cam));

    /* Simulate ~3 seconds at 60fps — velocities should decay to near zero */
    for (int i = 0; i < 180; i++)
        arpt_camera_update(cam, 1.0 / 60.0);

    TEST_ASSERT_FALSE(arpt_camera_is_moving(cam));
    arpt_camera_free(cam);
}

void test_zoom_at_center_matches_zoom(void) {
    arpt_camera *cam_a = arpt_camera_create();
    arpt_camera *cam_b = arpt_camera_create();
    arpt_camera_set_position(cam_a, DEG2RAD(7.4), DEG2RAD(46.9), 50000.0);
    arpt_camera_set_position(cam_b, DEG2RAD(7.4), DEG2RAD(46.9), 50000.0);
    arpt_camera_set_viewport(cam_a, 800, 600);
    arpt_camera_set_viewport(cam_b, 800, 600);

    /* Zoom at screen center should behave like plain zoom */
    arpt_camera_zoom_at(cam_a, 0.5, 400.0, 300.0);
    arpt_camera_zoom(cam_b, 0.5);

    /* Altitude should match exactly */
    TEST_ASSERT_DOUBLE_WITHIN(1.0, arpt_camera_altitude(cam_b),
                              arpt_camera_altitude(cam_a));
    /* Position should be very close (center ray hits near interest point) */
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.5),
                              arpt_camera_lon(cam_b),
                              arpt_camera_lon(cam_a));
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.5),
                              arpt_camera_lat(cam_b),
                              arpt_camera_lat(cam_a));

    arpt_camera_free(cam_a);
    arpt_camera_free(cam_b);
}

void test_fly_to_screen_activates_animation(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    /* Fly to screen center (should hit the globe) */
    arpt_camera_fly_to_screen(cam, 400.0, 300.0);
    TEST_ASSERT_TRUE(arpt_camera_is_moving(cam));
    arpt_camera_free(cam);
}

void test_animation_completes(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    arpt_camera_fly_to_screen(cam, 400.0, 300.0);
    TEST_ASSERT_TRUE(arpt_camera_is_moving(cam));

    /* Run enough frames to exceed the 0.4s animation duration */
    for (int i = 0; i < 60; i++)
        arpt_camera_update(cam, 1.0 / 60.0);

    TEST_ASSERT_FALSE(arpt_camera_is_moving(cam));
    /* Altitude should be approximately half the original */
    TEST_ASSERT_DOUBLE_WITHIN(50000.0, 250000.0, arpt_camera_altitude(cam));
    arpt_camera_free(cam);
}

void test_is_moving_false_at_rest(void) {
    arpt_camera *cam = arpt_camera_create();
    TEST_ASSERT_FALSE(arpt_camera_is_moving(cam));
    arpt_camera_free(cam);
}

/* ── Zoom-at stability ─────────────────────────────────────────────────── */

void test_zoom_at_no_drift(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(7.4), DEG2RAD(46.9), 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    /* Off-center cursor */
    double cx = 600.0, cy = 200.0;

    /* Record the globe point under cursor before any zoom */
    arpt_dvec3 origin, dir;
    TEST_ASSERT_TRUE(arpt_camera_screen_to_ray(cam, cx, cy, &origin, &dir));
    double t;
    TEST_ASSERT_TRUE(arpt_ray_ellipsoid(origin, dir, &t));
    arpt_dvec3 hit0 = arpt_dvec3_add(origin, arpt_dvec3_scale(dir, t));
    double anchor_lon, anchor_lat, anchor_alt;
    arpt_ecef_to_geodetic(hit0, &anchor_lon, &anchor_lat, &anchor_alt);

    /* Zoom in 50 times (scroll ticks) */
    for (int i = 0; i < 50; i++)
        arpt_camera_zoom_at(cam, 0.95, cx, cy);

    /* After all zooms, cast ray from same cursor position */
    TEST_ASSERT_TRUE(arpt_camera_screen_to_ray(cam, cx, cy, &origin, &dir));
    TEST_ASSERT_TRUE(arpt_ray_ellipsoid(origin, dir, &t));
    arpt_dvec3 hit_final = arpt_dvec3_add(origin, arpt_dvec3_scale(dir, t));
    double final_lon, final_lat, final_alt;
    arpt_ecef_to_geodetic(hit_final, &final_lon, &final_lat, &final_alt);

    /* The globe point under cursor should still be the anchor.
       0.01 degrees ≈ 1 km — well within acceptable drift for 50 ticks. */
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.01), anchor_lon, final_lon);
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.01), anchor_lat, final_lat);

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
    RUN_TEST(test_update_no_velocity_no_change);
    RUN_TEST(test_pan_velocity_decays);
    RUN_TEST(test_zoom_at_center_matches_zoom);
    RUN_TEST(test_fly_to_screen_activates_animation);
    RUN_TEST(test_animation_completes);
    RUN_TEST(test_is_moving_false_at_rest);
    RUN_TEST(test_zoom_at_no_drift);
    return UNITY_END();
}
