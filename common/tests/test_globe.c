#include "unity.h"
#include "globe.h"
#include <math.h>

#define DEG2RAD(d) ((d) * M_PI / 180.0)

void setUp(void) {}
void tearDown(void) {}

/* ── Geodetic → ECEF ───────────────────────────────────────────────────── */

void test_ecef_equator_prime_meridian(void) {
    /* (0, 0, 0) → X = a, Y = 0, Z = 0 */
    arpt_dvec3 p = arpt_geodetic_to_ecef(0.0, 0.0, 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, ARPT_WGS84_A, p.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.z);
}

void test_ecef_north_pole(void) {
    /* (0, 90°, 0) → X = 0, Y = 0, Z = b */
    arpt_dvec3 p = arpt_geodetic_to_ecef(0.0, DEG2RAD(90.0), 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, ARPT_WGS84_B, p.z);
}

void test_ecef_south_pole(void) {
    arpt_dvec3 p = arpt_geodetic_to_ecef(0.0, DEG2RAD(-90.0), 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -ARPT_WGS84_B, p.z);
}

void test_ecef_lon90(void) {
    /* (90°, 0, 0) → X = 0, Y = a, Z = 0 */
    arpt_dvec3 p = arpt_geodetic_to_ecef(DEG2RAD(90.0), 0.0, 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, ARPT_WGS84_A, p.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, p.z);
}

/* ── Roundtrip geodetic ↔ ECEF ─────────────────────────────────────────── */

void test_ecef_roundtrip(void) {
    double lon_in = DEG2RAD(7.4474);   /* Bern-ish */
    double lat_in = DEG2RAD(46.9480);
    double alt_in = 540.0;

    arpt_dvec3 ecef = arpt_geodetic_to_ecef(lon_in, lat_in, alt_in);
    double lon_out, lat_out, alt_out;
    arpt_ecef_to_geodetic(ecef, &lon_out, &lat_out, &alt_out);

    TEST_ASSERT_DOUBLE_WITHIN(1e-9, lon_in, lon_out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, lat_in, lat_out);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, alt_in, alt_out);  /* < 1mm */
}

void test_ecef_roundtrip_high_altitude(void) {
    double lon_in = DEG2RAD(-122.4194);
    double lat_in = DEG2RAD(37.7749);
    double alt_in = 35786000.0;  /* geostationary orbit */

    arpt_dvec3 ecef = arpt_geodetic_to_ecef(lon_in, lat_in, alt_in);
    double lon_out, lat_out, alt_out;
    arpt_ecef_to_geodetic(ecef, &lon_out, &lat_out, &alt_out);

    TEST_ASSERT_DOUBLE_WITHIN(1e-9, lon_in, lon_out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, lat_in, lat_out);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, alt_in, alt_out);
}

/* ── Surface normal ────────────────────────────────────────────────────── */

void test_surface_normal_equator(void) {
    arpt_dvec3 n = arpt_surface_normal(0.0, 0.0);
    /* At equator/prime meridian: normal points along +X */
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 1.0, n.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, n.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, n.z);
    /* Should be unit length */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, arpt_dvec3_len(n));
}

void test_surface_normal_north_pole(void) {
    arpt_dvec3 n = arpt_surface_normal(0.0, DEG2RAD(90.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, n.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, n.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 1.0, n.z);
}

/* ── Ray-ellipsoid ─────────────────────────────────────────────────────── */

void test_ray_hit(void) {
    /* Ray from 2× semi-major along +X, aiming at origin */
    arpt_dvec3 origin = {ARPT_WGS84_A * 2.0, 0.0, 0.0};
    arpt_dvec3 dir = {-1.0, 0.0, 0.0};
    double t;
    TEST_ASSERT_TRUE(arpt_ray_ellipsoid(origin, dir, &t));
    /* Should hit at X = a, so t ≈ a */
    TEST_ASSERT_DOUBLE_WITHIN(1.0, ARPT_WGS84_A, t);
}

void test_ray_miss(void) {
    /* Ray parallel to surface, will miss */
    arpt_dvec3 origin = {ARPT_WGS84_A * 2.0, 0.0, 0.0};
    arpt_dvec3 dir = {0.0, 1.0, 0.0};
    double t;
    TEST_ASSERT_FALSE(arpt_ray_ellipsoid(origin, dir, &t));
}

/* ── Globe rotation ────────────────────────────────────────────────────── */

void test_globe_rotation_maps_normal_to_neg_z(void) {
    /* R_globe maps the geodetic surface normal at the interest point to -Z. */
    double lon = DEG2RAD(7.4474);
    double lat = DEG2RAD(46.9480);

    arpt_dmat4 R = arpt_globe_rotation(lon, lat);
    arpt_dvec3 up = arpt_surface_normal(lon, lat);
    arpt_dvec3 cam = arpt_dmat4_rotate(R, up);

    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, cam.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, cam.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -1.0, cam.z);
}

void test_globe_rotation_ecef_mostly_neg_z(void) {
    /* The ECEF position should be mostly along -Z, with small x/y
       deviation due to ellipsoidal flattening. */
    double lon = DEG2RAD(7.4474);
    double lat = DEG2RAD(46.9480);

    arpt_dmat4 R = arpt_globe_rotation(lon, lat);
    arpt_dvec3 ecef = arpt_geodetic_to_ecef(lon, lat, 0.0);
    arpt_dvec3 cam = arpt_dmat4_transform(R, ecef);

    /* x should be exactly 0 (east component) */
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 0.0, cam.x);
    /* y deviation is < 22 km due to ellipsoidal flattening */
    TEST_ASSERT_TRUE(fabs(cam.y) < 25000.0);
    TEST_ASSERT_TRUE(cam.z < 0.0);
}

void test_globe_rotation_is_orthonormal(void) {
    arpt_dmat4 R = arpt_globe_rotation(DEG2RAD(45.0), DEG2RAD(30.0));
    /* Columns should be unit vectors and orthogonal */
    arpt_dvec3 c0 = {R.m[0], R.m[1], R.m[2]};
    arpt_dvec3 c1 = {R.m[4], R.m[5], R.m[6]};
    arpt_dvec3 c2 = {R.m[8], R.m[9], R.m[10]};
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, arpt_dvec3_len(c0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, arpt_dvec3_len(c1));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, arpt_dvec3_len(c2));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, arpt_dvec3_dot(c0, c1));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, arpt_dvec3_dot(c0, c2));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, arpt_dvec3_dot(c1, c2));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ecef_equator_prime_meridian);
    RUN_TEST(test_ecef_north_pole);
    RUN_TEST(test_ecef_south_pole);
    RUN_TEST(test_ecef_lon90);
    RUN_TEST(test_ecef_roundtrip);
    RUN_TEST(test_ecef_roundtrip_high_altitude);
    RUN_TEST(test_surface_normal_equator);
    RUN_TEST(test_surface_normal_north_pole);
    RUN_TEST(test_ray_hit);
    RUN_TEST(test_ray_miss);
    RUN_TEST(test_globe_rotation_maps_normal_to_neg_z);
    RUN_TEST(test_globe_rotation_ecef_mostly_neg_z);
    RUN_TEST(test_globe_rotation_is_orthonormal);
    return UNITY_END();
}
