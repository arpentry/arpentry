#include "unity.h"
#include "math3d.h"

#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* dvec3 */

void test_dvec3_add(void) {
    arpt_dvec3 a = {1, 2, 3}, b = {4, 5, 6};
    arpt_dvec3 r = arpt_dvec3_add(a, b);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 5.0, r.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 7.0, r.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 9.0, r.z);
}

void test_dvec3_sub(void) {
    arpt_dvec3 a = {4, 5, 6}, b = {1, 2, 3};
    arpt_dvec3 r = arpt_dvec3_sub(a, b);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 3.0, r.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 3.0, r.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 3.0, r.z);
}

void test_dvec3_scale(void) {
    arpt_dvec3 v = {2, 3, 4};
    arpt_dvec3 r = arpt_dvec3_scale(v, 2.5);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 5.0, r.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 7.5, r.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 10.0, r.z);
}

void test_dvec3_dot(void) {
    arpt_dvec3 a = {1, 0, 0}, b = {0, 1, 0};
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, arpt_dvec3_dot(a, b));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 1.0, arpt_dvec3_dot(a, a));
}

void test_dvec3_cross(void) {
    arpt_dvec3 x = {1, 0, 0}, y = {0, 1, 0};
    arpt_dvec3 z = arpt_dvec3_cross(x, y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, z.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, z.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 1.0, z.z);
}

void test_dvec3_normalize(void) {
    arpt_dvec3 v = {3, 0, 4};
    arpt_dvec3 n = arpt_dvec3_normalize(v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, arpt_dvec3_len(n));
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.6, n.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, n.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.8, n.z);
}

void test_dvec3_normalize_zero(void) {
    arpt_dvec3 v = {0, 0, 0};
    arpt_dvec3 n = arpt_dvec3_normalize(v);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, n.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, n.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, n.z);
}

/* dmat4 */

void test_dmat4_identity_multiply(void) {
    arpt_dmat4 id = arpt_dmat4_identity();
    arpt_dmat4 a =
        arpt_dmat4_from_cols((arpt_dvec3){1, 0, 0}, (arpt_dvec3){0, 1, 0},
                             (arpt_dvec3){0, 0, 1}, (arpt_dvec3){5, 6, 7});
    arpt_dmat4 r = arpt_dmat4_mul(id, a);
    for (int i = 0; i < 16; i++)
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, a.m[i], r.m[i]);
}

void test_dmat4_transform(void) {
    arpt_dmat4 t =
        arpt_dmat4_from_cols((arpt_dvec3){1, 0, 0}, (arpt_dvec3){0, 1, 0},
                             (arpt_dvec3){0, 0, 1}, (arpt_dvec3){10, 20, 30});
    arpt_dvec3 p = {1, 2, 3};
    arpt_dvec3 r = arpt_dmat4_transform(t, p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 11.0, r.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 22.0, r.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 33.0, r.z);
}

void test_dmat4_rotate(void) {
    /* 90-degree rotation around Z: X→Y, Y→-X */
    arpt_dmat4 rz =
        arpt_dmat4_from_cols((arpt_dvec3){0, 1, 0}, (arpt_dvec3){-1, 0, 0},
                             (arpt_dvec3){0, 0, 1}, (arpt_dvec3){99, 99, 99});
    arpt_dvec3 d = arpt_dmat4_rotate(rz, (arpt_dvec3){1, 0, 0});
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, d.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, d.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, d.z);
}

/* mat4 perspective */

/* NDC depth of a view-space point `z_view` metres in front of the camera. */
static float ndc_depth(arpt_mat4 p, float z_view) {
    float z = p.m[10] * (-z_view) + p.m[14];
    float w = p.m[11] * (-z_view) + p.m[15];
    return z / w;
}

void test_perspective_depth_range(void) {
    /* Reversed-Z, infinite far: near maps to 1, infinity to 0, and nearer
       is always greater. The `far` argument is ignored. */
    arpt_mat4 p =
        arpt_mat4_perspective((float)(M_PI / 4.0), 1.0f, 1.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, ndc_depth(p, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, ndc_depth(p, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1e-7f, ndc_depth(p, 1e7f));
    TEST_ASSERT_TRUE(ndc_depth(p, 2.0f) > ndc_depth(p, 3.0f));
    /* Behind the camera the clip w goes negative (the cull the label placer
       relies on). */
    TEST_ASSERT_TRUE(p.m[11] * (-(-5.0f)) + p.m[15] < 0.0f);
}

void test_orthographic_depth_range(void) {
    /* Reversed-Z, finite: near → 1, far → 0, linear; behind the near plane
       the depth runs past 1, which is what the label cull tests for. */
    arpt_mat4 p = arpt_mat4_orthographic(-1, 1, -1, 1, 10.0f, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, ndc_depth(p, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ndc_depth(p, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, ndc_depth(p, 505.0f));
    TEST_ASSERT_TRUE(ndc_depth(p, -5.0f) > 1.0f);
}

/* The premise of the reversed-Z change, as arithmetic, with the camera's
   own near/far rule (near = max(1, alt/100), far = alt + 2R). What it shows
   is narrower than "24 bits cannot see a kerb": from 3 km up the forward
   buffer's quantum at 2 km eye distance is ~8 mm, fine. The forward mapping
   fails where near is pinned at 1 m — under 100 m altitude, looking along
   the street at grazing tilt — where its quantum at 2 km is ~0.24 m, twice
   the 0.12 m kerb. Reversed-Z on a float holds ~1e-7 of the eye distance in
   both cases. Measured the way the GPU stores it: the next representable
   depth around the point's own. */
void test_reversed_z_resolves_a_kerb_at_range(void) {
    const float eye_m = 2000.0f;
    const float alts[2] = {100.0f, 3000.0f};
    const float fwd_expect[2] = {0.12f, 0.02f}; /* forward quantum bounds */
    for (int i = 0; i < 2; i++) {
        float near = fmaxf(1.0f, alts[i] * 0.01f);
        float far = alts[i] + 2.0f * 6378137.0f;
        arpt_mat4 p =
            arpt_mat4_perspective((float)(M_PI / 4.0), 1.0f, near, far);
        float d = ndc_depth(p, eye_m);
        /* z_ndc = near / z, so dz = z^2 / near · d(depth). */
        float quantum_m = (eye_m * eye_m / near) * (nextafterf(d, 1.0f) - d);
        TEST_ASSERT_TRUE_MESSAGE(quantum_m < 0.002f,
                                 "reversed-Z quantum at 2 km must be < 2 mm");
        /* Forward z_ndc = (far·(z − near)) / (z·(far − near)): dz/d(depth) =
           z^2 (far − near) / (near·far) over a fixed 2^-24 quantum. */
        float fwd_q = (eye_m * eye_m * (far - near) / (near * far)) / 16777216.0f;
        if (i == 0)
            TEST_ASSERT_TRUE_MESSAGE(fwd_q > fwd_expect[0],
                "forward 24-bit at 100 m altitude was coarser than a kerb");
        else
            TEST_ASSERT_TRUE_MESSAGE(fwd_q < fwd_expect[1],
                "forward 24-bit at 3 km altitude already resolved a kerb");
    }
}

/* Conversions */

void test_dmat4_to_mat4(void) {
    arpt_dmat4 d = arpt_dmat4_identity();
    d.m[12] = 1234567.890123;
    arpt_mat4 f = arpt_dmat4_to_mat4(d);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)d.m[12], f.m[12]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, f.m[0]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_dvec3_add);
    RUN_TEST(test_dvec3_sub);
    RUN_TEST(test_dvec3_scale);
    RUN_TEST(test_dvec3_dot);
    RUN_TEST(test_dvec3_cross);
    RUN_TEST(test_dvec3_normalize);
    RUN_TEST(test_dvec3_normalize_zero);
    RUN_TEST(test_dmat4_identity_multiply);
    RUN_TEST(test_dmat4_transform);
    RUN_TEST(test_dmat4_rotate);
    RUN_TEST(test_perspective_depth_range);
    RUN_TEST(test_orthographic_depth_range);
    RUN_TEST(test_reversed_z_resolves_a_kerb_at_range);
    RUN_TEST(test_dmat4_to_mat4);
    return UNITY_END();
}
