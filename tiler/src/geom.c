/* Core geometry operations. */

#include "geom.h"

#include <stdlib.h>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#elif defined(__SSE2__)
#include <emmintrin.h>
#endif

void arpt_geom_bbox(const arpt_geom *g, double bbox[4]) {
    if (!g || g->n_coords == 0) {
        bbox[0] = bbox[1] = bbox[2] = bbox[3] = 0.0;
        return;
    }

    uint32_t n = g->n_coords;
    const double *x = g->x;
    const double *y = g->y;

#if defined(__ARM_NEON)
    float64x2_t vmin_x = vdupq_n_f64(x[0]);
    float64x2_t vmax_x = vdupq_n_f64(x[0]);
    float64x2_t vmin_y = vdupq_n_f64(y[0]);
    float64x2_t vmax_y = vdupq_n_f64(y[0]);

    uint32_t i = 1;
    uint32_t end2 = 1 + ((n - 1) & ~1u);
    for (; i < end2; i += 2) {
        float64x2_t vx = vld1q_f64(x + i);
        float64x2_t vy = vld1q_f64(y + i);
        vmin_x = vminq_f64(vmin_x, vx);
        vmax_x = vmaxq_f64(vmax_x, vx);
        vmin_y = vminq_f64(vmin_y, vy);
        vmax_y = vmaxq_f64(vmax_y, vy);
    }

    /* Reduce 2-wide lanes to scalar */
    double mn_x = vminvq_f64(vmin_x);
    double mx_x = vmaxvq_f64(vmax_x);
    double mn_y = vminvq_f64(vmin_y);
    double mx_y = vmaxvq_f64(vmax_y);

    /* Handle remaining element */
    for (; i < n; i++) {
        if (x[i] < mn_x) mn_x = x[i];
        if (x[i] > mx_x) mx_x = x[i];
        if (y[i] < mn_y) mn_y = y[i];
        if (y[i] > mx_y) mx_y = y[i];
    }

    bbox[0] = mn_x; bbox[1] = mn_y;
    bbox[2] = mx_x; bbox[3] = mx_y;

#elif defined(__SSE2__)
    __m128d vmin_x = _mm_set1_pd(x[0]);
    __m128d vmax_x = _mm_set1_pd(x[0]);
    __m128d vmin_y = _mm_set1_pd(y[0]);
    __m128d vmax_y = _mm_set1_pd(y[0]);

    uint32_t i = 1;
    uint32_t end2 = 1 + ((n - 1) & ~1u);
    for (; i < end2; i += 2) {
        __m128d vx = _mm_loadu_pd(x + i);
        __m128d vy = _mm_loadu_pd(y + i);
        vmin_x = _mm_min_pd(vmin_x, vx);
        vmax_x = _mm_max_pd(vmax_x, vx);
        vmin_y = _mm_min_pd(vmin_y, vy);
        vmax_y = _mm_max_pd(vmax_y, vy);
    }

    double tmp[2];
    _mm_storeu_pd(tmp, vmin_x); double mn_x = tmp[0] < tmp[1] ? tmp[0] : tmp[1];
    _mm_storeu_pd(tmp, vmax_x); double mx_x = tmp[0] > tmp[1] ? tmp[0] : tmp[1];
    _mm_storeu_pd(tmp, vmin_y); double mn_y = tmp[0] < tmp[1] ? tmp[0] : tmp[1];
    _mm_storeu_pd(tmp, vmax_y); double mx_y = tmp[0] > tmp[1] ? tmp[0] : tmp[1];

    for (; i < n; i++) {
        if (x[i] < mn_x) mn_x = x[i];
        if (x[i] > mx_x) mx_x = x[i];
        if (y[i] < mn_y) mn_y = y[i];
        if (y[i] > mx_y) mx_y = y[i];
    }

    bbox[0] = mn_x; bbox[1] = mn_y;
    bbox[2] = mx_x; bbox[3] = mx_y;

#else
    bbox[0] = bbox[2] = x[0];
    bbox[1] = bbox[3] = y[0];
    for (uint32_t i = 1; i < n; i++) {
        if (x[i] < bbox[0]) bbox[0] = x[i];
        if (x[i] > bbox[2]) bbox[2] = x[i];
        if (y[i] < bbox[1]) bbox[1] = y[i];
        if (y[i] > bbox[3]) bbox[3] = y[i];
    }
#endif
}

void arpt_geom_free(arpt_geom *g) {
    if (!g) return;
    free(g->x);
    free(g->y);
    free(g->z);
    free(g->offsets);
}
