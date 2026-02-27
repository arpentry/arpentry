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
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 5.0,  r.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 7.5,  r.y);
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
    arpt_dmat4 a = arpt_dmat4_from_cols(
        (arpt_dvec3){1, 0, 0}, (arpt_dvec3){0, 1, 0},
        (arpt_dvec3){0, 0, 1}, (arpt_dvec3){5, 6, 7});
    arpt_dmat4 r = arpt_dmat4_mul(id, a);
    for (int i = 0; i < 16; i++)
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, a.m[i], r.m[i]);
}

void test_dmat4_transform(void) {
    arpt_dmat4 t = arpt_dmat4_from_cols(
        (arpt_dvec3){1, 0, 0}, (arpt_dvec3){0, 1, 0},
        (arpt_dvec3){0, 0, 1}, (arpt_dvec3){10, 20, 30});
    arpt_dvec3 p = {1, 2, 3};
    arpt_dvec3 r = arpt_dmat4_transform(t, p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 11.0, r.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 22.0, r.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 33.0, r.z);
}

void test_dmat4_rotate(void) {
    /* 90-degree rotation around Z: X→Y, Y→-X */
    arpt_dmat4 rz = arpt_dmat4_from_cols(
        (arpt_dvec3){0, 1, 0}, (arpt_dvec3){-1, 0, 0},
        (arpt_dvec3){0, 0, 1}, (arpt_dvec3){99, 99, 99});
    arpt_dvec3 d = arpt_dmat4_rotate(rz, (arpt_dvec3){1, 0, 0});
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, d.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, d.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, d.z);
}

/* mat4 perspective */

void test_perspective_depth_range(void) {
    /* WebGPU: near maps to z=0, far maps to z=1 */
    arpt_mat4 p = arpt_mat4_perspective((float)(M_PI / 4.0), 1.0f, 1.0f, 100.0f);
    /* Point at near plane: (0, 0, -near, 1) → clip z should map to 0 */
    float z_near = p.m[10] * (-1.0f) + p.m[14];
    float w_near = p.m[11] * (-1.0f);
    float ndc_near = z_near / w_near;
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, ndc_near);

    /* Point at far plane: (0, 0, -far, 1) → clip z should map to 1 */
    float z_far = p.m[10] * (-100.0f) + p.m[14];
    float w_far = p.m[11] * (-100.0f);
    float ndc_far = z_far / w_far;
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ndc_far);
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
    RUN_TEST(test_dmat4_to_mat4);
    return UNITY_END();
}
