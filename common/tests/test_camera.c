#include "unity.h"
#include "camera.h"
#include "globe.h"

#include <math.h>

#define DEG2RAD(d) ((d) * M_PI / 180.0)

void setUp(void) {}
void tearDown(void) {}

/* Defaults */

void test_camera_defaults(void) {
    arpt_camera *cam = arpt_camera_create();
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_lon(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_lat(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 10000000.0, arpt_camera_altitude(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_tilt(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_camera_bearing(cam));
    arpt_camera_free(cam);
}

/* Projection */

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

/* Tile model matrix */

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

/* Globe rotation correctness */

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

/* Pan */

void test_pan_begin_stores_anchor(void) {
    /* pan_begin + pan_move at the same point → camera should not move */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(7.0), DEG2RAD(47.0), 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    double lon0 = arpt_camera_lon(cam);
    double lat0 = arpt_camera_lat(cam);

    arpt_camera_pan_begin(cam, 400.0, 300.0);
    arpt_camera_pan_move(cam, 400.0, 300.0);

    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.01), lon0, arpt_camera_lon(cam));
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.01), lat0, arpt_camera_lat(cam));

    arpt_camera_free(cam);
}

void test_pan_move_shifts_camera(void) {
    /* pan_begin at center, pan_move offset → camera moves */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(7.0), DEG2RAD(47.0), 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    double lon0 = arpt_camera_lon(cam);
    double lat0 = arpt_camera_lat(cam);

    arpt_camera_pan_begin(cam, 400.0, 300.0);
    arpt_camera_pan_move(cam, 450.0, 350.0);

    /* Camera should have moved — lon/lat changed */
    double dlon = fabs(arpt_camera_lon(cam) - lon0);
    double dlat = fabs(arpt_camera_lat(cam) - lat0);
    TEST_ASSERT_TRUE(dlon > DEG2RAD(0.01) || dlat > DEG2RAD(0.01));

    arpt_camera_free(cam);
}

void test_pan_linear_north(void) {
    /* pan(0, -100) → latitude increases (moving map south = camera north) */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, DEG2RAD(45.0), 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    double lat0 = arpt_camera_lat(cam);
    arpt_camera_pan(cam, 0.0, -100.0);

    TEST_ASSERT_TRUE(arpt_camera_lat(cam) > lat0);
    arpt_camera_free(cam);
}

/* Zoom */

void test_zoom_at_center(void) {
    /* Zoom at screen center with zero tilt → lon/lat unchanged, alt scaled */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(10.0), DEG2RAD(45.0), 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    double lon0 = arpt_camera_lon(cam);
    double lat0 = arpt_camera_lat(cam);

    arpt_camera_zoom_at(cam, 400.0, 300.0, 0.5);

    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.5), lon0, arpt_camera_lon(cam));
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.5), lat0, arpt_camera_lat(cam));
    TEST_ASSERT_DOUBLE_WITHIN(1000.0, 250000.0, arpt_camera_altitude(cam));

    arpt_camera_free(cam);
}

void test_zoom_at_corner(void) {
    /* Zoom at corner: the point under the corner stays ~fixed */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, DEG2RAD(10.0), DEG2RAD(45.0), 500000.0);
    arpt_camera_set_viewport(cam, 800, 600);

    /* Get the geodetic point under the corner before zoom */
    double corner_lon_before, corner_lat_before;
    bool hit_before = arpt_camera_screen_to_geodetic(
        cam, 200.0, 150.0, &corner_lon_before, &corner_lat_before);
    TEST_ASSERT_TRUE(hit_before);

    arpt_camera_zoom_at(cam, 200.0, 150.0, 0.5);

    /* Same screen point should hit near same geodetic coords */
    double corner_lon_after, corner_lat_after;
    bool hit_after = arpt_camera_screen_to_geodetic(
        cam, 200.0, 150.0, &corner_lon_after, &corner_lat_after);
    TEST_ASSERT_TRUE(hit_after);
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.5), corner_lon_before,
                              corner_lon_after);
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.5), corner_lat_before,
                              corner_lat_after);

    arpt_camera_free(cam);
}

/* Tilt / Bearing */

void test_tilt_bearing_basic(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_tilt_bearing(cam, DEG2RAD(30.0), DEG2RAD(90.0));

    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.1), DEG2RAD(30.0),
                              arpt_camera_tilt(cam));
    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.1), DEG2RAD(90.0),
                              arpt_camera_bearing(cam));

    arpt_camera_free(cam);
}

void test_tilt_bearing_clamp(void) {
    /* Tilt should clamp at 60° */
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_tilt_bearing(cam, DEG2RAD(90.0), 0.0);

    TEST_ASSERT_DOUBLE_WITHIN(DEG2RAD(0.1), DEG2RAD(60.0),
                              arpt_camera_tilt(cam));

    arpt_camera_free(cam);
}

/* Zoom level */

void test_zoom_level_high_altitude(void) {
    arpt_camera *cam = arpt_camera_create();
    arpt_camera_set_position(cam, 0.0, 0.0, 20000000.0); /* 20,000 km */
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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_camera_defaults);
    RUN_TEST(test_projection_near_far);
    RUN_TEST(test_tile_model_at_interest);
    RUN_TEST(test_tile_model_nearby_tile);
    RUN_TEST(test_screen_center_ray_along_neg_z);
    RUN_TEST(test_pan_begin_stores_anchor);
    RUN_TEST(test_pan_move_shifts_camera);
    RUN_TEST(test_pan_linear_north);
    RUN_TEST(test_zoom_at_center);
    RUN_TEST(test_zoom_at_corner);
    RUN_TEST(test_tilt_bearing_basic);
    RUN_TEST(test_tilt_bearing_clamp);
    RUN_TEST(test_zoom_level_high_altitude);
    RUN_TEST(test_zoom_level_low_altitude);
    RUN_TEST(test_zoom_level_clamped);
    return UNITY_END();
}
