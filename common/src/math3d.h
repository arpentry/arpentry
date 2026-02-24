#ifndef ARPENTRY_MATH3D_H
#define ARPENTRY_MATH3D_H

#include <math.h>
#include <string.h>

/* ── Float32 types (GPU upload) ────────────────────────────────────────── */

typedef struct { float x, y, z; } arpt_vec3;

/* Column-major 4x4 matrix: m[col*4 + row], matching WGSL mat4x4<f32>. */
typedef struct { float m[16]; } arpt_mat4;

/* ── Float64 types (CPU precision) ─────────────────────────────────────── */

typedef struct { double x, y, z; } arpt_dvec3;

/* Column-major 4x4 matrix in double precision. */
typedef struct { double m[16]; } arpt_dmat4;

/* ── dvec3 operations ──────────────────────────────────────────────────── */

static inline arpt_dvec3 arpt_dvec3_add(arpt_dvec3 a, arpt_dvec3 b) {
    return (arpt_dvec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline arpt_dvec3 arpt_dvec3_sub(arpt_dvec3 a, arpt_dvec3 b) {
    return (arpt_dvec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline arpt_dvec3 arpt_dvec3_scale(arpt_dvec3 v, double s) {
    return (arpt_dvec3){v.x * s, v.y * s, v.z * s};
}

static inline double arpt_dvec3_dot(arpt_dvec3 a, arpt_dvec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline arpt_dvec3 arpt_dvec3_cross(arpt_dvec3 a, arpt_dvec3 b) {
    return (arpt_dvec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static inline double arpt_dvec3_len(arpt_dvec3 v) {
    return sqrt(arpt_dvec3_dot(v, v));
}

static inline arpt_dvec3 arpt_dvec3_normalize(arpt_dvec3 v) {
    double len = arpt_dvec3_len(v);
    if (len < 1e-15) return (arpt_dvec3){0, 0, 0};
    return arpt_dvec3_scale(v, 1.0 / len);
}

/* ── dmat4 operations ──────────────────────────────────────────────────── */

static inline arpt_dmat4 arpt_dmat4_identity(void) {
    arpt_dmat4 r;
    memset(&r, 0, sizeof(r));
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0;
    return r;
}

/* Build a 4x4 from 3x3 rotation columns + translation. */
static inline arpt_dmat4 arpt_dmat4_from_cols(arpt_dvec3 cx, arpt_dvec3 cy,
                                               arpt_dvec3 cz, arpt_dvec3 t) {
    arpt_dmat4 r;
    memset(&r, 0, sizeof(r));
    /* Column 0 */
    r.m[0] = cx.x;  r.m[1] = cx.y;  r.m[2] = cx.z;
    /* Column 1 */
    r.m[4] = cy.x;  r.m[5] = cy.y;  r.m[6] = cy.z;
    /* Column 2 */
    r.m[8] = cz.x;  r.m[9] = cz.y;  r.m[10] = cz.z;
    /* Column 3 (translation) */
    r.m[12] = t.x;  r.m[13] = t.y;  r.m[14] = t.z;  r.m[15] = 1.0;
    return r;
}

static inline arpt_dmat4 arpt_dmat4_mul(arpt_dmat4 a, arpt_dmat4 b) {
    arpt_dmat4 r;
    for (int c = 0; c < 4; c++) {
        for (int row = 0; row < 4; row++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++)
                sum += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = sum;
        }
    }
    return r;
}

/* Transform a point (w=1) by a dmat4. */
static inline arpt_dvec3 arpt_dmat4_transform(arpt_dmat4 m, arpt_dvec3 v) {
    return (arpt_dvec3){
        m.m[0]*v.x + m.m[4]*v.y + m.m[8]*v.z  + m.m[12],
        m.m[1]*v.x + m.m[5]*v.y + m.m[9]*v.z  + m.m[13],
        m.m[2]*v.x + m.m[6]*v.y + m.m[10]*v.z + m.m[14],
    };
}

/* Rotate a direction (w=0) by a dmat4's upper-3x3. */
static inline arpt_dvec3 arpt_dmat4_rotate(arpt_dmat4 m, arpt_dvec3 v) {
    return (arpt_dvec3){
        m.m[0]*v.x + m.m[4]*v.y + m.m[8]*v.z,
        m.m[1]*v.x + m.m[5]*v.y + m.m[9]*v.z,
        m.m[2]*v.x + m.m[6]*v.y + m.m[10]*v.z,
    };
}

/* ── mat4 operations ───────────────────────────────────────────────────── */

/**
 * Perspective projection for WebGPU (z clip = [0, 1]).
 * fov_y in radians, aspect = width / height.
 */
static inline arpt_mat4 arpt_mat4_perspective(float fov_y, float aspect,
                                               float near, float far) {
    float f = 1.0f / tanf(fov_y * 0.5f);
    arpt_mat4 r;
    memset(&r, 0, sizeof(r));
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    /* WebGPU NDC z = [0, 1]: maps near → 0, far → 1 */
    r.m[10] = far / (near - far);
    r.m[11] = -1.0f;
    r.m[14] = (near * far) / (near - far);
    return r;
}

/* ── Conversions ───────────────────────────────────────────────────────── */

static inline arpt_mat4 arpt_dmat4_to_mat4(arpt_dmat4 d) {
    arpt_mat4 r;
    for (int i = 0; i < 16; i++)
        r.m[i] = (float)d.m[i];
    return r;
}

static inline arpt_vec3 arpt_dvec3_to_vec3(arpt_dvec3 d) {
    return (arpt_vec3){(float)d.x, (float)d.y, (float)d.z};
}

#endif /* ARPENTRY_MATH3D_H */
