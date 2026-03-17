#include "unity.h"
#include "simplify.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

void setUp(void) {}
void tearDown(void) {}

static void test_single_point(void) {
    double x[] = {1.0};
    double y[] = {2.0};
    TEST_ASSERT_EQUAL_UINT32(1, arpt_simplify(x, y, 1, 1.0));
}

static void test_two_points(void) {
    double x[] = {0.0, 1.0};
    double y[] = {0.0, 1.0};
    TEST_ASSERT_EQUAL_UINT32(2, arpt_simplify(x, y, 2, 1.0));
}

static void test_collinear_removed(void) {
    /* Three collinear points: middle one should be removed */
    double x[] = {0.0, 1.0, 2.0};
    double y[] = {0.0, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 3, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, x[1]);
}

static void test_collinear_many(void) {
    /* Many collinear points reduced to endpoints */
    double x[10], y[10];
    for (int i = 0; i < 10; i++) {
        x[i] = (double)i;
        y[i] = 0.0;
    }
    uint32_t n = arpt_simplify(x, y, 10, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
}

static void test_zigzag_preserved(void) {
    /* Zigzag with large deviations should be mostly preserved */
    double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double y[] = {0.0, 10.0, 0.0, 10.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 5, 1.0);
    TEST_ASSERT_EQUAL_UINT32(5, n);
}

static void test_tolerance_zero(void) {
    /* Zero tolerance keeps all points */
    double x[] = {0.0, 1.0, 2.0, 3.0};
    double y[] = {0.0, 0.1, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 4, 0.0);
    TEST_ASSERT_EQUAL_UINT32(4, n);
}

static void test_large_tolerance(void) {
    /* Very large tolerance removes all intermediate points */
    double x[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    double y[] = {0.0, 0.01, -0.01, 0.01, 0.0};
    uint32_t n = arpt_simplify(x, y, 5, 100.0);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, x[1]);
}

static void test_triangle(void) {
    /* Triangle with one point far from baseline */
    double x[] = {0.0, 1.0, 2.0};
    double y[] = {0.0, 5.0, 0.0};
    /* tolerance smaller than deviation: keep all */
    uint32_t n = arpt_simplify(x, y, 3, 1.0);
    TEST_ASSERT_EQUAL_UINT32(3, n);

    /* Reset and use large tolerance */
    double x2[] = {0.0, 1.0, 2.0};
    double y2[] = {0.0, 0.5, 0.0};
    n = arpt_simplify(x2, y2, 3, 1.0);
    TEST_ASSERT_EQUAL_UINT32(2, n);
}

static void test_endpoints_always_kept(void) {
    double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double y[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    uint32_t n = arpt_simplify(x, y, 5, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, x[1]);
}

/* ---- Ring simplification tests ---- */

static void test_ring_closed_square(void) {
    /* Closed square ring: 4 unique + closing = 5 vertices.
     * With zero tolerance, all vertices should be preserved. */
    double x[] = {0.0, 10.0, 10.0, 0.0, 0.0};
    double y[] = {0.0, 0.0, 10.0, 10.0, 0.0};
    uint32_t n = arpt_simplify_ring(x, y, 5, 0.0);
    TEST_ASSERT_EQUAL_UINT32(5, n);
    /* Ring should still be closed */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x[0], x[n - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, y[0], y[n - 1]);
}

static void test_ring_preserves_shape(void) {
    /* Closed ring approximating a circle with 16 vertices.
     * The old DP (with first==last) would measure distance-from-origin
     * and poorly simplify vertices near vertex 0.  The ring-aware DP
     * should preserve the shape symmetrically. */
    const uint32_t n = 16;
    double x[17], y[17];
    for (uint32_t i = 0; i < n; i++) {
        double angle = 2.0 * M_PI * (double)i / (double)n;
        x[i] = 10.0 * cos(angle);
        y[i] = 10.0 * sin(angle);
    }
    x[n] = x[0]; y[n] = y[0]; /* close ring */

    uint32_t out = arpt_simplify_ring(x, y, n + 1, 1.0);
    /* Must keep at least a valid ring (3 unique + closing = 4) */
    TEST_ASSERT_TRUE(out >= 4);
    /* Ring must still be closed */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x[0], x[out - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, y[0], y[out - 1]);

    /* All surviving vertices should be at radius ~10 (not pushed inward) */
    for (uint32_t i = 0; i < out - 1; i++) {
        double r = sqrt(x[i] * x[i] + y[i] * y[i]);
        TEST_ASSERT_DOUBLE_WITHIN(0.1, 10.0, r);
    }
}

static void test_ring_symmetric_simplification(void) {
    /* A closed ring where vertex 0 is at the bottom.  The old DP
     * would aggressively remove vertices near the bottom while
     * preserving the top.  Ring-aware DP should simplify both
     * halves equally. */
    double x[] = {0.0,  5.0, 10.0, 10.0, 10.0, 5.0, 0.0, 0.0, 0.0};
    double y[] = {5.0,  0.1, 5.0,  7.0,  10.0, 9.9, 10.0, 7.0, 5.0};
    /* 8 unique + closing = 9 vertices.
     * Vertices (5,0.1) and (5,9.9) are nearly collinear with their
     * neighbours and should be removed symmetrically. */
    uint32_t out = arpt_simplify_ring(x, y, 9, 0.5);
    /* Both near-collinear vertices should be removed */
    TEST_ASSERT_TRUE(out <= 7);
    /* Ring must still be closed */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x[0], x[out - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, y[0], y[out - 1]);
}

static void test_ring_not_closed_fallback(void) {
    /* If the ring is not closed, arpt_simplify_ring should fall back
     * to standard open polyline simplification. */
    double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double y[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    uint32_t n = arpt_simplify_ring(x, y, 5, 0.001);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, x[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, x[1]);
}

static void test_ring_minimal(void) {
    /* Triangle ring (3 unique + closing = 4): nothing to simplify */
    double x[] = {0.0, 10.0, 5.0, 0.0};
    double y[] = {0.0, 0.0, 10.0, 0.0};
    uint32_t n = arpt_simplify_ring(x, y, 4, 100.0);
    TEST_ASSERT_EQUAL_UINT32(4, n);
}

/* Helper: check if segments (x[i],y[i])-(x[i+1],y[i+1]) and
 * (x[j],y[j])-(x[j+1],y[j+1]) properly cross. */
static bool ring_self_intersects(const double *x, const double *y,
                                  uint32_t n_unique) {
    /* Check all pairs of non-adjacent ring segments */
    for (uint32_t i = 0; i < n_unique; i++) {
        uint32_t i1 = (i + 1) % n_unique;
        for (uint32_t j = i + 2; j < n_unique; j++) {
            if (i == 0 && j == n_unique - 1) continue; /* adjacent at closure */
            uint32_t j1 = (j + 1) % n_unique;

            double ax = x[i],  ay = y[i];
            double bx = x[i1], by = y[i1];
            double cx = x[j],  cy = y[j];
            double dx = x[j1], dy = y[j1];

            /* Orientation test for proper crossing */
            double d1 = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
            double d2 = (bx-ax)*(dy-ay) - (by-ay)*(dx-ax);
            double d3 = (dx-cx)*(ay-cy) - (dy-cy)*(ax-cx);
            double d4 = (dx-cx)*(by-cy) - (dy-cy)*(bx-cx);

            if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
                return true;
        }
    }
    return false;
}

static void test_ring_topology_strait(void) {
    /* Simulate the Spain-Africa scenario: a concave ring with a narrow
     * strait.  Standard DP would bridge the strait; topology-preserving
     * DP must refuse to flatten across it.
     *
     * Shape (viewed from above, north up):
     *
     *   8--7--6--5
     *   |        |
     *   9  strait 4
     *   |        |
     *  10--11-12-3
     *       |  |
     *      13--2
     *       |  |
     *      14--1
     *       |  |
     *       0/15
     *
     * The narrow gap between vertices 9-10-11 and 3-4 is the "strait".
     * Standard DP with large tolerance would connect 8→5 directly,
     * bridging across the strait.  Topology-preserving DP must keep
     * enough vertices around the strait to prevent self-intersection. */

    double x[] = {
        5.0,   /* 0  bottom center */
        6.0,   /* 1 */
        6.0,   /* 2 */
        8.0,   /* 3  right side of strait */
        8.0,   /* 4 */
        8.0,   /* 5  top right */
        6.0,   /* 6 */
        4.0,   /* 7 */
        2.0,   /* 8  top left */
        2.0,   /* 9  left side of strait */
        2.0,   /* 10 */
        4.0,   /* 11 */
        4.0,   /* 12 */
        4.0,   /* 13 */
        4.0,   /* 14 */
        5.0,   /* 15 = 0 closing */
    };
    double y[] = {
        0.0,   /* 0 */
        0.0,   /* 1 */
        2.0,   /* 2 */
        2.0,   /* 3  right side of strait */
        5.0,   /* 4 */
        8.0,   /* 5 */
        8.0,   /* 6 */
        8.0,   /* 7 */
        8.0,   /* 8  top left */
        5.0,   /* 9  left side of strait */
        2.0,   /* 10 */
        2.0,   /* 11 */
        2.0,   /* 12 */
        1.0,   /* 13 */
        0.0,   /* 14 */
        0.0,   /* 15 = 0 closing */
    };

    uint32_t count = 16; /* 15 unique + closing */
    uint32_t out = arpt_simplify_ring(x, y, count, 2.0);

    /* Ring must still be closed */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x[0], x[out - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, y[0], y[out - 1]);

    /* The simplified ring must NOT self-intersect */
    uint32_t n_unique = out - 1;
    TEST_ASSERT_FALSE(ring_self_intersects(x, y, n_unique));
}

static void test_ring_topology_hourglass(void) {
    /* An hourglass-like ring where the waist is very narrow.
     * Without topology checks, DP could collapse the waist and
     * create a self-intersection (bowtie). */
    double x[] = {
        0.0, 10.0, 5.1, 10.0, 0.0, 4.9, 0.0
    };
    double y[] = {
        0.0, 0.0, 5.0, 10.0, 10.0, 5.0, 0.0
    };
    /* 6 unique + closing = 7 */
    uint32_t out = arpt_simplify_ring(x, y, 7, 2.0);

    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x[0], x[out - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, y[0], y[out - 1]);

    uint32_t n_unique = out - 1;
    TEST_ASSERT_FALSE(ring_self_intersects(x, y, n_unique));
}

static void test_shared_edge_consistency(void) {
    /* Two polygon rings sharing an edge A-B-C-D-E must retain the same
     * vertices along that edge after independent simplification.
     *
     * Ring 1: ...private1... - A - B - C - D - E - ...private1...
     * Ring 2: ...private2... - E - D - C - B - A - ...private2...
     *                          (reversed, as adjacent polygon winding)
     *
     * The shared edge has some vertices that should be simplified away
     * and others that should be kept. Both rings must agree. */

    /* Shared edge: (0,0) - (1,0.01) - (2,0) - (3,0.01) - (4,0) */
    /* Vertices (1,0.01) and (3,0.01) are nearly collinear, should be removed */

    /* Ring 1: square-ish polygon above the shared edge */
    double x1[] = {0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 2.0, 0.0, 0.0};
    double y1[] = {0.0, 0.01, 0.0, 0.01, 0.0, 5.0, 5.5, 5.0, 0.0};
    /* 8 unique + closing = 9 */

    /* Ring 2: polygon below the shared edge (edge reversed) */
    double x2[] = {4.0, 3.0, 2.0, 1.0, 0.0, 0.0, 2.0, 4.0, 4.0};
    double y2[] = {0.0, 0.01, 0.0, 0.01, 0.0, -5.0, -5.5, -5.0, 0.0};
    /* 8 unique + closing = 9 */

    uint32_t n1 = arpt_simplify_ring(x1, y1, 9, 0.1);
    uint32_t n2 = arpt_simplify_ring(x2, y2, 9, 0.1);

    /* Both rings must still be closed */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x1[0], x1[n1 - 1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, x2[0], x2[n2 - 1]);

    /* Extract the shared edge vertices from each ring.
     * In ring 1, the shared edge goes forward: find vertices with y ~ 0.
     * In ring 2, the shared edge goes backward: find vertices with y ~ 0.
     * Both should have the same set of (x,y) pairs. */
    double shared1_x[10], shared1_y[10];
    uint32_t shared1_n = 0;
    for (uint32_t i = 0; i < n1 - 1; i++) {
        if (fabs(y1[i]) < 0.1) {
            shared1_x[shared1_n] = x1[i];
            shared1_y[shared1_n] = y1[i];
            shared1_n++;
        }
    }

    double shared2_x[10], shared2_y[10];
    uint32_t shared2_n = 0;
    for (uint32_t i = 0; i < n2 - 1; i++) {
        if (fabs(y2[i]) < 0.1) {
            shared2_x[shared2_n] = x2[i];
            shared2_y[shared2_n] = y2[i];
            shared2_n++;
        }
    }

    /* Same number of shared edge vertices */
    TEST_ASSERT_EQUAL_UINT32(shared1_n, shared2_n);

    /* Same coordinates (ring 2's edge is reversed) */
    for (uint32_t i = 0; i < shared1_n; i++) {
        uint32_t j = shared2_n - 1 - i;
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, shared1_x[i], shared2_x[j]);
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, shared1_y[i], shared2_y[j]);
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_single_point);
    RUN_TEST(test_two_points);
    RUN_TEST(test_collinear_removed);
    RUN_TEST(test_collinear_many);
    RUN_TEST(test_zigzag_preserved);
    RUN_TEST(test_tolerance_zero);
    RUN_TEST(test_large_tolerance);
    RUN_TEST(test_triangle);
    RUN_TEST(test_endpoints_always_kept);
    RUN_TEST(test_ring_closed_square);
    RUN_TEST(test_ring_preserves_shape);
    RUN_TEST(test_ring_symmetric_simplification);
    RUN_TEST(test_ring_not_closed_fallback);
    RUN_TEST(test_ring_minimal);
    RUN_TEST(test_ring_topology_strait);
    RUN_TEST(test_ring_topology_hourglass);
    RUN_TEST(test_shared_edge_consistency);
    return UNITY_END();
}
